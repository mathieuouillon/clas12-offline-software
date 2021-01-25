package org.jlab.clas.tracking.kalmanfilter.helical;

import java.util.*;

import Jama.Matrix;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.jlab.geom.prim.Point3D;
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
    private double resiCut = 100; // residual cut for the measurements
    public int totNumIter = 1; // Number of iteration of the Kalman Filter
    public Map<Integer, HitOnTrack> TrjPoints = new HashMap<>();
    public Helix KFHelix;
    public double chi2 = 0;
    public int NDF = 0;
    public double TotalEnergyLoss = 0;
    public double NIST_TotalEnergyLoss = 0;


    /**
     * A extended Kalman filter using as state vector a=(d_rho, phi_0, omega, z0, tanL)^T as define in
     * this paper : https://www-jlc.kek.jp/subg/offl/kaltest/doc/ReferenceManual.pdf.
     * Use a 4th order Runge-Kutta algorithm for extrapolate x,y,z between each measSurface.
     *
     * @param helix        Helix given by global fit for starting the Kalman Filter.
     * @param cov          Covariance matrix define as :
     *                     | d0*d0          phi0 * d0      omega * d0        0         0      |
     *                     | phi0 * d0      phi0 * phi0    phi0 * omega      0         0      |
     *                     | omega * d0     phi0 * omega   omega * omega     0         0      |
     *                     |    0                0               0        z0 * z0      0      |
     *                     |    0                0               0           0    tanL * tanL |
     * @param swimmer      Propagate a charged particle into a magnetic field by using Runge-Kutta 4th differential equation solver
     * @param Xb           X component of beam axis in mm
     * @param Yb           Y component of beam axis in mm
     * @param Zref         Target shift in mm
     * @param measSurfaces Surfaces define for each measurement (here a cylinder and a strip(line))
     */
    public KFitter(Helix helix, Matrix cov, Swim swimmer, double Xb, double Yb,
                   double Zref, List<Surface> measSurfaces) {
        _Xb = Xb;
        _Yb = Yb;
        _tarShift = Zref;
        this.init(helix, cov, swimmer, Xb, Yb, Zref, mv, measSurfaces);
    }

    /**
     * Initialise the Kalman filter by using the helix and the covariance matrix
     *
     * @param helix        Helix define in org.jlab.clas.tracking.trackrep.Helix;
     * @param cov          Covariance matrix define in the constructor come from the global fit
     * @param swimmer      Propagate a charged particle into a magnetic field by using Runge-Kutta 4th differential equation solver
     * @param Xb           X component of beam axis in mm
     * @param Yb           Y component of beam axis in mm
     * @param Zref         Target shift in mm
     * @param mv           Measurement vector list
     * @param measSurfaces Surfaces define for each measurement (here a cylinder and a strip(line))
     */
    public void init(Helix helix, Matrix cov, Swim swimmer,
                     double Xb, double Yb, double Zref, MeasVecs mv,
                     List<Surface> measSurfaces) {

        sv.shift = Zref;
        mv.setMeasVecs(measSurfaces);

        if (sv.X0 != null) {
            sv.X0.clear();
        } else {
            sv.X0 = new ArrayList<>();
        }
        if (sv.Y0 != null) {
            sv.Y0.clear();
        } else {
            sv.Y0 = new ArrayList<>();
        }
        if (sv.Z0 != null) {
            sv.Z0.clear();
        } else {
            sv.Z0 = new ArrayList<>();
        }

        sv.X0.add(Xb);
        sv.Y0.add(Yb);
        sv.Z0.add(0.0);

        for (int i = 1; i < mv.measurements.size(); i++) {
            sv.X0.add(Xb);
            sv.Y0.add(Yb);
            sv.Z0.add(0.0);
        }

        sv.init(helix, cov, swimmer);
        this.NDF = mv.measurements.size() - 6;
    }

    /**
     * Run the Kalman filter with @totNumIter the number of iteration of the Kalman Filter.
     * We take the one with the best chi square
     *
     * @param swimmer Propagate a charged particle into a magnetic field by using Runge-Kutta 4th differential equation solver
     * TODO : change filter when direction is -1 to be a smoother cf paper in the constructor
     */
    public void runFitter(Swim swimmer) {
        double newchisq = Double.POSITIVE_INFINITY;
        this.NDF = sv.X0.size() - 5;

        for (int it = 0; it < totNumIter; it++) {
            this.chi2 = 0;
            for (int k = 0; k < sv.X0.size() - 1; k++) {
                if (sv.trackCov.get(k) == null || mv.measurements.get(k + 1) == null) {
                    return;
                }
                sv.transport(k, k + 1, sv.trackTraj.get(k), sv.trackCov.get(k), mv.measurements.get(k + 1), swimmer, 1);
                TotalEnergyLoss += sv.trackTraj.get(k).Eloss;
                NIST_TotalEnergyLoss += sv.trackTraj.get(k).NIST_Eloss;
                this.filter(k + 1, swimmer, 1);
            }

            for (int k = sv.X0.size() - 1; k > 0; k--) {
                if (sv.trackCov.get(k) == null || mv.measurements.get(k - 1) == null) {
                    return;
                }
                sv.transport(k, k - 1, sv.trackTraj.get(k), sv.trackCov.get(k), mv.measurements.get(k - 1), swimmer, -1);
                if (k > 1) this.filter(k - 1, swimmer, -1);
            }

            this.chi2 = this.calc_chi2(swimmer);
            if (this.chi2 < newchisq) {
                newchisq = this.chi2;
                KFHelix = sv.setTrackPars();
                finalStateVec = sv.trackTraj.get(0);
                setFitFailed = false;
            } else {
                this.chi2 = newchisq;
                break;
            }
        }
    }

    /**
     * Calculate the chi square of the Kalman Filter.
     *
     * @param swimmer Propagate a charged particle into a magnetic field by using Runge-Kutta 4th differential equation solver
     * @return double chi square
     */
    // TODO : refaire le calcul du chi square probablement faux
    private double calc_chi2(Swim swimmer) {
        double chi2 = 0;
        int ndf = -5;
        StateVec stv = sv.transported(0, 1, sv.trackTraj.get(0), mv.measurements.get(1), swimmer, 1);
        double dh = mv.dh(1, stv);
        if (!mv.measurements.get(1).skip) {
            chi2 = dh * dh / mv.measurements.get(1).error;
            ndf++;
        }
        for (int k = 1; k < sv.X0.size() - 1; k++) {
            if (!mv.measurements.get(k + 1).skip) {
                stv = sv.transported(k, k + 1, stv, mv.measurements.get(k + 1), swimmer, 1);
                dh = mv.dh(k + 1, stv);
                chi2 += dh * dh / mv.measurements.get(k + 1).error;
                ndf++;
            }
        }
        return chi2;
    }

    /**
     * Update the state vector and the covariance matrix as explain in the paper. (cf constructor)
     *
     * @param k       step of the Kalman Filter
     * @param swimmer cf constructor
     * @param dir     direction
     */
    private void filter(int k, Swim swimmer, int dir) {
        if (sv.trackTraj.get(k) != null && sv.trackCov.get(k).covMat != null && !mv.measurements.get(k).skip) {
            RealMatrix V = MatrixUtils.createRealMatrix(1, 1); // Noise covariance matrix
            V.addToEntry(0, 0, mv.measurements.get(k).error);
            RealMatrix C = sv.trackCov.get(k).covMat; // Covariance matrix
            double[] H_table = mv.H(sv.trackTraj.get(k), sv, mv.measurements.get(k), swimmer, dir);
            RealMatrix H = new Array2DRowRealMatrix(H_table); // Jacobian matrix of the function h
            H = H.transpose();
            RealMatrix Inter = MatrixUtils.inverse(V.add(H.multiply(C.multiply(H.transpose()))));
            RealMatrix K = C.multiply(H.transpose().multiply(Inter)); // Kalman gain matrix

            // Extrapolated state vector
            double drho_filt = sv.trackTraj.get(k).d_rho;
            double phi0_filt = sv.trackTraj.get(k).phi0;
            double kappa_filt = sv.trackTraj.get(k).kappa;
            double dz_filt = sv.trackTraj.get(k).dz;
            double tanL_filt = sv.trackTraj.get(k).tanL;

            double dh = mv.dh(k, sv.trackTraj.get(k)); // measurement residual -> distance

            // Update of the state vector
            if (!Double.isNaN(dh)) {
                drho_filt = drho_filt + K.getEntry(0, 0) * dh;
                phi0_filt = phi0_filt + K.getEntry(1, 0) * dh;
                kappa_filt = kappa_filt + K.getEntry(2, 0) * dh;
                dz_filt = dz_filt + K.getEntry(3, 0) * dh;
                tanL_filt = tanL_filt + K.getEntry(4, 0) * dh;
            }

            // Update of the covariance matrix
            RealMatrix I = MatrixUtils.createRealIdentityMatrix(5);
            RealMatrix C_filt = (I.subtract(K.multiply(H))).multiply(C);

            StateVec fVec = new StateVec(sv.trackTraj.get(k).k);


            // Record the update step 
            fVec.d_rho = drho_filt;
            fVec.phi0 = phi0_filt;
            fVec.kappa = kappa_filt;
            fVec.dz = dz_filt;
            fVec.tanL = tanL_filt;
            fVec.alpha = sv.trackTraj.get(k).alpha;

            double dh_filt = mv.dh(k, fVec);
            if (Math.abs(dh_filt) < Math.abs(dh) && Math.abs(dh_filt) / Math.sqrt(V.getEntry(0, 0)) < this.getResiCut()) {
                sv.trackTraj.get(k).d_rho = drho_filt;
                sv.trackTraj.get(k).phi0 = phi0_filt;
                sv.trackTraj.get(k).kappa = kappa_filt;
                sv.trackTraj.get(k).dz = dz_filt;
                sv.trackTraj.get(k).tanL = tanL_filt;
                sv.trackTraj.get(k).phi = fVec.phi;
                sv.trackTraj.get(k).x = fVec.x;
                sv.trackTraj.get(k).y = fVec.y;
                sv.trackTraj.get(k).z = fVec.z;
                sv.trackCov.get(k).covMat = C_filt;
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
            System.out.println(C.get(k, 0) + "	" + C.get(k, 1) + "	" + C.get(k, 2) + "	" + C.get(k, 3) + "	" + C.get(k, 4));
        }
        System.out.println("    ");
    }

    /**
     * Test if @param mat is non singular (invertible : if B exist such that AB = BA = I)
     * Should be remove if the matrix library is change
     *
     * @param mat Matrix A
     * @return boolean True if mat is non singular or False if not
     */
    private boolean isNonsingular(Matrix mat) {
        double matDet = mat.det();
        return !(Math.abs(matDet) < 1.e-30);
    }

    /**
     * Print d_rho, phi_0, kappa, dz, tanL, x, y, z, phi and theta for @param sv -- used for debugging
     *
     * @param sv State vector
     */
    private void printStateVec(StateVec sv) {
        System.out.println(sv.k + "]  ");
        System.out.println((float) sv.d_rho + ", " +
                (float) sv.phi0 + ", " +
                (float) sv.kappa + ", " +
                (float) sv.dz + ", " +
                (float) sv.tanL + " xyz " + new Point3D(sv.x, sv.y, sv.z) + " phi " + Math.toDegrees(Math.atan2(sv.y, sv.x)) + " theta " + Math.toDegrees(Math.atan2(sv.y, sv.z - sv.dz)));
        System.out.println("  ");
    }

    public static class HitOnTrack {

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
