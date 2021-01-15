package org.jlab.clas.tracking.kalmanfilter.helical;

import java.lang.reflect.Array;
import java.util.*;

import Jama.Matrix;

import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;
import org.jlab.io.base.DataEvent;
import org.jlab.clas.swimtools.Swim;
import org.jlab.clas.tracking.kalmanfilter.Surface;
import org.jlab.clas.tracking.kalmanfilter.helical.StateVecs.StateVec;
import org.jlab.clas.tracking.trackrep.Helix;

public class KFitter {
    
    public static int polarity = 1;
    public boolean setFitFailed = true;
    StateVecs sv = new StateVecs();
    MeasVecs mv = new MeasVecs();
    public StateVec finalStateVec;
    private double _tarShift; //targetshift
    private double _Xb; //beam axis pars
    private double _Yb;
    private double resiCut = 100;//residual cut for the measurements
    public int totNumIter = 5; // Number of iteration of the Kalman Filter
    public Map<Integer, HitOnTrack> TrjPoints = new HashMap<Integer, HitOnTrack>();
    public Helix KFHelix;
    public double chi2 = 0;
    public int NDF = 0;


    /**
     *
     * @param helix Helix given by global fit for starting the Kalman Filter.
     * @param cov Covariance matrix define by :
     *            | d0*d0          phi0 * d0      omega * d0        0         0      |
     *            | phi0 * d0      phi0 * phi0    phi0 * omega      0         0      |
     *            | omega * d0     phi0 * omega   omega * omega     0         0      |
     *            |    0                0               0        z0 * z0      0      |
     *            |    0                0               0           0    tanL * tanL |
     * @param event
     * @param swimmer
     * @param Xb X component of beam axis
     * @param Yb Y component of beam axis
     * @param Zref
     * @param measSurfaces Surfaces define for each measurement (here a cylinder and a strip(line))
     */
    public KFitter(Helix helix, Matrix cov, DataEvent event, Swim swimmer, double Xb, double Yb,
                   double Zref, List<Surface> measSurfaces) {
        _Xb = Xb;
        _Yb = Yb;
        _tarShift = Zref;
        this.init(helix, cov, event, swimmer, Xb, Yb, Zref, mv, measSurfaces);
    }

    public void init(Helix helix, Matrix cov, DataEvent event, Swim swimmer,
                     double Xb, double Yb, double Zref, MeasVecs mv,
                     List<Surface> measSurfaces) {

        sv.shift = Zref;
        mv.setMeasVecs(measSurfaces);

        if (sv.X0 != null) {
            sv.X0.clear();
        } else {
            sv.X0 = new ArrayList<Double>();
        }
        if (sv.Y0 != null) {
            sv.Y0.clear();
        } else {
            sv.Y0 = new ArrayList<Double>();
        }
        if (sv.Z0 != null) {
            sv.Z0.clear();
        } else {
            sv.Z0 = new ArrayList<Double>();
        }

        sv.X0.add(Xb);
        sv.Y0.add(Yb);
        sv.Z0.add(0.0);

        for (int i = 1; i < mv.measurements.size(); i++) {
            sv.X0.add(Xb);
            sv.Y0.add(Yb);
            sv.Z0.add(0.0);
        }

        sv.init(helix, cov, this, swimmer);
        // this.NDF = mv.measurements.size() - 6;
    }
                            
    public void runFitter(Swim swimmer) {
        double newchisq = Double.POSITIVE_INFINITY;
        this.NDF = sv.X0.size()-5; 
        
        for (int it = 0; it < totNumIter; it++) {
            this.chi2 = 0;
            for (int k = 0; k < sv.X0.size() - 1; k++) {
                if (sv.trackCov.get(k) == null || mv.measurements.get(k + 1) == null) {return;}
                sv.transport(k, k + 1, sv.trackTraj.get(k), sv.trackCov.get(k), mv.measurements.get(k+1), swimmer);
                this.filter(k + 1, swimmer, 1); 
            }
            
            for (int k =  sv.X0.size() - 1; k>0 ;k--) {
                if (sv.trackCov.get(k) == null || mv.measurements.get(k - 1) == null) {return;}
                sv.transport(k, k - 1, sv.trackTraj.get(k), sv.trackCov.get(k), mv.measurements.get(k-1), swimmer);
                if(k>1) this.filter(k - 1, swimmer, -1);
            }

            this.chi2=this.calc_chi2(swimmer);
            if(this.chi2<newchisq) { 
                newchisq=this.chi2;
                KFHelix = sv.setTrackPars();
                finalStateVec = sv.trackTraj.get(0);
                setFitFailed = false;
            }
            else {
                this.chi2 =newchisq ;
                break;
            }
        }
    }

    private double calc_chi2(Swim swimmer) {
        double chi2 =0;
        int ndf = -5;
        StateVec stv = sv.transported(0, 1, sv.trackTraj.get(0), mv.measurements.get(1), swimmer);
        double dh = mv.dh(1, stv);
        if(!mv.measurements.get(1).skip) {
            chi2 = dh*dh / mv.measurements.get(1).error;
            ndf++;
        }
        for(int k = 1; k< sv.X0.size()-1; k++) {
            if(!mv.measurements.get(k + 1).skip) {
                stv = sv.transported(k, k+1, stv, mv.measurements.get(k+1), swimmer);
                dh = mv.dh(k+1, stv);
                chi2 += dh*dh / mv.measurements.get(k+1).error;
                ndf++;
            }
        }  
        return chi2;
    }

    public double deltaETot = 0;
    
    private void filter(int k, Swim swimmer, int dir) {
        if (sv.trackTraj.get(k) != null && sv.trackCov.get(k).covMat != null && !mv.measurements.get(k).skip) {
            double[] K = new double[5]; // Gain matrix
            Arrays.fill(K,0.0);
            double V = mv.measurements.get(k).error; // measurement error covariance matrix

            double dh = mv.dh(k, sv.trackTraj.get(k));
            
            //get the projector Matrix
            double[] H = mv.H(sv.trackTraj.get(k), sv,  mv.measurements.get(k), swimmer, dir);
            double[][] HTGH = new double[][]{
                {H[0] * H[0] / V, H[0] * H[1] / V, H[0] * H[2] / V, H[0] * H[3] / V, H[0] * H[4] / V},
                {H[1] * H[0] / V, H[1] * H[1] / V, H[1] * H[2] / V, H[1] * H[3] / V, H[1] * H[4] / V},
                {H[2] * H[0] / V, H[2] * H[1] / V, H[2] * H[2] / V, H[2] * H[3] / V, H[2] * H[4] / V},
                {H[3] * H[0] / V, H[3] * H[1] / V, H[3] * H[2] / V, H[3] * H[3] / V, H[3] * H[4] / V},
                {H[4] * H[0] / V, H[4] * H[1] / V, H[4] * H[2] / V, H[4] * H[3] / V, H[4] * H[4] / V}
            };

            Matrix Ci = null;

            if (!this.isNonsingular(sv.trackCov.get(k).covMat)) {
                return;
            }
            try {
                Ci = sv.trackCov.get(k).covMat.inverse();
            } catch (Exception e) {
                return;
            }
            Matrix Ca = null;
            try {
                Ca = Ci.plus(new Matrix(HTGH));
            } catch (Exception e) {
                return;
            }
            if (Ca != null && !this.isNonsingular(Ca)) {
                return;
            }
            if (Ca != null && this.isNonsingular(Ca)) {
                if (Ca.inverse() != null) {
                    sv.trackCov.get(k).covMat = Ca.inverse();
                } else {
                    return;
                }
            }
            else {
                return;
            }
            
            for (int j = 0; j < 5; j++) {
                for (int i = 0; i < 5; i++) {
                    K[j] += H[i] * sv.trackCov.get(k).covMat.get(j, i) / V;
                }
            }

            double drho_filt = sv.trackTraj.get(k).d_rho;
            double phi0_filt = sv.trackTraj.get(k).phi0;
            double kappa_filt = sv.trackTraj.get(k).kappa;
            double dz_filt = sv.trackTraj.get(k).dz;
            double tanL_filt = sv.trackTraj.get(k).tanL;

            // System.out.println("Before : drho = " + drho_filt + " phi0 = " + phi0_filt + " kappa = " + kappa_filt + " dz = " + dz_filt + " tanL = " + tanL_filt);

            if (!Double.isNaN(dh)) {
                drho_filt -= K[0] * dh;
                phi0_filt -= K[1] * dh;
                kappa_filt -= K[2] * dh;
                dz_filt -= K[3] * dh;
                tanL_filt -= K[4] * dh;
            }

            // TODO add energy loss here :
            //Compute the distance between k and k+1
            Point3D start = new Point3D(sv.trackTraj.get(k-1).x, sv.trackTraj.get(k-1).y, sv.trackTraj.get(k-1).z);
            Point3D end = new Point3D(sv.trackTraj.get(k).x, sv.trackTraj.get(k).y, sv.trackTraj.get(k).z);
            double distance = start.distance(end);

            //Compute the momentum at k :
            Vector3D P = sv.P(k-1);
            double p = P.mag(); // MeV
            double mass = 938.27208816; // proton
            double c = 0.299792458;
            double beta = p / Math.sqrt(p * p + mass * mass); // particle momentum
            double m_ec2 = 0.5109989461; // MeV
            double K_ = 0.307075; // Mev mol-1 cm2
            double z = 1; // charge number of the proton
            double gamma = 1/(Math.sqrt(1-beta*beta));
            double W_max = 2*m_ec2*beta*beta*gamma*gamma;

            double X = Math.log(beta*gamma);
            // For Helium :
            double X0 = 2.202; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
            double X1 = 4.0; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
            double a = 0.0114; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
            double m = 7.625; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
            double C = 11.139; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
            double Z = 2; // atomic number of helium
            double I = 10*Z; // eV
            double A = 4.002602; // g/mol
            double rho = 0.1664*0.001; // density of helium at 20°C and 1 atm in g/cm3 found on https://www.engineeringtoolbox.com/gas-density-d_158.html
            double deltaX = 0;
            if(X>X0 && X<X1) {deltaX = 4.6052*X + a*Math.pow((X1-X),m) + C;}
            else if(X>X1) {deltaX = 4.6052*X + C;}
            double deltaE1 = K_*z*z*Z/A*1/(beta*beta)*(0.5*Math.log((2*m_ec2*beta*beta*gamma*gamma*W_max)/(I*I)) - beta*beta - deltaX/2)*distance*0.1*rho;

            // For CO2 :
            X0 = 1.629; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
            X1 = 4.0; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
            a = 0.1944; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
            m = 3.027; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
            C = 10.154; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
            A = 44.01; // g/mol
            Z = 22; // atomic number of CO2
            rho = 1.842*0.001; // density of CO2 at 20°C and 1 atm in g/cm3 found on https://www.engineeringtoolbox.com/gas-density-d_158.html
            if(X>X0 && X<X1) {deltaX = 4.6052*X + a*Math.pow((X1-X),m) + C;}
            else if(X>X1) {deltaX = 4.6052*X + C;}
            double deltaE2 = K_*z*z*Z/A*1/(beta*beta)*(0.5*Math.log((2*m_ec2*beta*beta*gamma*gamma*W_max)/(I*I)) - beta*beta - deltaX/2)*distance*0.1*rho;

            double w = 4*4.002602 + 44.01;
            double w1 = 4*4.002602/w; // weight fraction of the 1th element (helium)
            double w2 = 44.01/w; // weight fraction of the 2th element (CO2)
            double deltaE = w1*deltaE1 + w2*deltaE2;
            deltaETot += deltaE;

            StateVec fVec = sv.new StateVec(sv.trackTraj.get(k).k);

            fVec.d_rho = drho_filt;
            fVec.phi0 = phi0_filt;
            fVec.kappa = kappa_filt;
            fVec.dz = dz_filt;
            fVec.tanL = tanL_filt;
            fVec.alpha = sv.trackTraj.get(k).alpha;
            sv.setStateVecPosAtMeasSite(k, fVec, mv.measurements.get(k), swimmer);

            double dh_filt = mv.dh(k, fVec);
            if (Math.abs(dh_filt) < Math.abs(dh)
                    && Math.abs(dh_filt) / Math.sqrt(V) < this.getResiCut()) {
                sv.trackTraj.get(k).d_rho = drho_filt;
                sv.trackTraj.get(k).phi0 = phi0_filt;
                sv.trackTraj.get(k).kappa = kappa_filt;
                sv.trackTraj.get(k).dz = dz_filt;
                sv.trackTraj.get(k).tanL = tanL_filt;
                sv.trackTraj.get(k).phi = fVec.phi;
                sv.trackTraj.get(k).x = fVec.x;
                sv.trackTraj.get(k).y = fVec.y;
                sv.trackTraj.get(k).z = fVec.z;
            } else {
                this.NDF--;
                mv.measurements.get(k).skip = true;
            }
        }
    }

    /**
     * @return the resiCut
     */
    public double getResiCut() {
        return resiCut;
    }

    /**
     * @param resiCut the resiCut to set
     */
    public void setResiCut(double resiCut) {
        this.resiCut = resiCut;
    }

    /**
     * @return the _Xb
     */
    public double getXb() {
        return _Xb;
    }

    /**
     * @param _Xb the _Xb to set
     */
    public void setXb(double _Xb) {
        this._Xb = _Xb;
    }

    /**
     * @return the _Yb
     */
    public double getYb() {
        return _Yb;
    }

    /**
     * @param _Yb the _Yb to set
     */
    public void setYb(double _Yb) {
        this._Yb = _Yb;
    }

    /**
     * @return the _tarShift
     */
    public double getTarShift() {
        return _tarShift;
    }

    /**
     * @param _TarShift the _tarShift to set
     */
    public void setTarShift(double _TarShift) {
        this._tarShift = _TarShift;
    }

    /**
     * prints the matrix -- used for debugging
     *
     * @param C matrix
     */
    public void printMatrix(Matrix C) {
        System.out.println("    ");
        for (int k = 0; k < 5; k++) {
            System.out.println(C.get(k, 0)+"	"+C.get(k, 1)+"	"+C.get(k, 2)+"	"+C.get(k, 3)+"	"+C.get(k, 4));
        }
        System.out.println("    ");
    }

    private boolean isNonsingular(Matrix mat) {
        double matDet = mat.det();
        return !(Math.abs(matDet) < 1.e-30);
    }

    private void printStateVec(StateVec sv) {
        System.out.println(sv.k+"]  ");
        System.out.println((float)sv.d_rho+", "+
            (float)sv.phi0+", "+
            (float)sv.kappa+", "+
            (float)sv.dz+", "+
            (float)sv.tanL+" xyz "+new Point3D(sv.x,sv.y,sv.z)+" phi "+Math.toDegrees(Math.atan2(sv.y,sv.x))+" theta "+Math.toDegrees(Math.atan2(sv.y,sv.z-sv.dz)));
        System.out.println("  ");
    }

    public class HitOnTrack {

        public int layer;
        public double x;
        public double y;
        public double z;
        public double px;
        public double py;
        public double pz;
        public boolean isMeasUsed = true;
        
        HitOnTrack(int layer, double x, double y, double z, double px, double py, double pz) {
            this.layer = layer;
            this.x = x;
            this.y = y;
            this.z = z;
            this.px = px;
            this.py = py;
            this.pz = pz;

        }
    }

}
