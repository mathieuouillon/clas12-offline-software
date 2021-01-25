package org.jlab.clas.tracking.kalmanfilter.helical;

import java.util.*;

import Jama.Matrix;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.jlab.geom.prim.Vector3D;
import org.jlab.geom.prim.Point3D;
import org.jlab.clas.swimtools.Swim;
import org.jlab.clas.tracking.kalmanfilter.helical.MeasVecs.MeasVec;
import org.jlab.clas.tracking.trackrep.Helix;

public class StateVecs {

    private Helix util;
    public double units;
    public double lightVel;
    public boolean straight;
    public Map<Integer, StateVec> trackTraj = new HashMap<>();
    public Map<Integer, CovMat> trackCov = new HashMap<>();
    public StateVec initSV = new StateVec(0);
    public StateVec StateVec;
    public List<Double> X0;
    public List<Double> Y0;
    public List<Double> Z0; // reference points
    public double shift; // target shift
    double[] value = new double[8]; // x,y,z,px,py,pz,phi,pathlenght
    double[] swimPars = new double[8];
    B Bf = new B(0);

    public double[] getStateVecPosAtMeasSite(int k, StateVec iVec, MeasVec mv, Swim swim, boolean useSwimmer) {
        this.resetArrays(swimPars);
        this.resetArrays(value);

        Point3D ps = new Point3D(0, 0, 0);

        StateVec kVec = new StateVec(k);
        kVec.phi0 = iVec.phi0;
        kVec.d_rho = iVec.d_rho;
        kVec.kappa = iVec.kappa;
        kVec.dz = iVec.dz;
        kVec.tanL = iVec.tanL;
        kVec.alpha = iVec.alpha;

        if (mv.surface != null) {
            double x = X0.get(0) + kVec.d_rho * Math.cos(kVec.phi0);
            double y = Y0.get(0) + kVec.d_rho * Math.sin(kVec.phi0);
            double z = Z0.get(0) + kVec.dz;

            Bf.swimmer = swim;
            Bf.x = x;
            Bf.y = y;
            Bf.z = z;
            Bf.set();
            kVec.alpha = Bf.alpha;

            if (k == 0) {
                value[0] = x;
                value[1] = y;
                value[2] = z;
                value[3] = 0.0;
                return value;
            }

            if (this.straight) {
                Vector3D u = new Vector3D(-(Math.signum(kVec.kappa)) * Math.sin(kVec.phi0),
                        (Math.signum(kVec.kappa)) * Math.cos(kVec.phi0),
                        (Math.signum(kVec.kappa)) * kVec.tanL).asUnit();

                if (mv.surface.plane != null) {
                    double alpha = (mv.surface.finitePlaneCorner2.y() - mv.surface.finitePlaneCorner1.y()) /
                            (mv.surface.finitePlaneCorner2.x() - mv.surface.finitePlaneCorner1.x());
                    double l = (alpha * (x - mv.surface.finitePlaneCorner1.x()) - (y - mv.surface.finitePlaneCorner1.y())) / (u.y() - alpha * u.x());

                    kVec.x = x + l * u.x();
                    kVec.y = y + l * u.y();
                    kVec.z = z + l * u.z();

                }
                if (mv.surface.cylinder != null) {
                    double r = 0.5 * (mv.surface.cylinder.baseArc().radius() + mv.surface.cylinder.highArc().radius());
                    double delta = Math.sqrt((x * u.x() + y * u.y()) * (x * u.x() + y * u.y()) - (-r * r + x * x + y * y) * (u.x() * u.x() + u.y() * u.y()));
                    double l = (-(x * u.x() + y * u.y()) + delta) / (u.x() * u.x() + u.y() * u.y());
                    double phi = Math.atan2(trackTraj.get(k - 1).y, trackTraj.get(k - 1).x);
                    double phiref = Math.atan2(y + l * u.y(), x + l * u.x());

                    if (Math.abs(phiref - phi) > Math.PI / 2) {
                        l = (-(x * u.x() + y * u.y()) - delta) / (u.x() * u.x() + u.y() * u.y());
                    }

                    kVec.x = x + l * u.x();
                    kVec.y = y + l * u.y();
                    kVec.z = z + l * u.z();

                }
                value[0] = kVec.x;
                value[1] = kVec.y;
                value[2] = kVec.z;
                value[3] = this.calcPhi(kVec);

                return value;
            }
            if (mv.surface.plane != null) {
                //Update B
                double r0 = mv.surface.finitePlaneCorner1.toVector3D().dot(mv.surface.plane.normal());
                double stepSize = 5; //mm
                int nSteps = (int) (r0 / stepSize);

                double dist = 0;

                for (int i = 1; i < nSteps; i++) {
                    dist = (double) (i * stepSize);
                    this.setHelixPars(kVec, swim);
                    ps = util.getHelixPointAtR(dist);
                    kVec.x = ps.x();
                    kVec.y = ps.y();
                    kVec.z = ps.z();
                    this.tranState(k, kVec, swim);
                }
                this.setHelixPars(kVec, swim);
                ps = util.getHelixPointAtPlane(mv.surface.finitePlaneCorner1.x(), mv.surface.finitePlaneCorner1.y(),
                        mv.surface.finitePlaneCorner2.x(), mv.surface.finitePlaneCorner2.y(), 10);
                this.tranState(k, kVec, swim);
                kVec.x = ps.x();
                kVec.y = ps.y();
                kVec.z = ps.z();
                if (swimPars == null)
                    return null;
                swimPars[0] = ps.x();
                swimPars[1] = ps.y();
                swimPars[2] = ps.z();

//                    x = X0.get(0) + kVec.d_rho * Math.cos(kVec.phi0);
//                    y = Y0.get(0) + kVec.d_rho * Math.sin(kVec.phi0);
//                    z = Z0.get(0) + kVec.dz;
//                    Bf = new B(kVec.k, x, y, z, swim);
//                    kVec.alpha = Bf.alpha;
//                    this.setTrackPars(kVec, swim);
//                    swimPars = swim.SwimToPlaneBoundary(mv.surface.plane.point().toVector3D().dot(mv.surface.plane.normal())/units,
//                            mv.surface.plane.normal(), 1);
//                    if(swimPars==null)
//                        return null;
//                    for(int j =0; j < 3; j++) {
//                        swimPars[j]*=units;
//                    }
//                    kVec.x = swimPars[0];
//                    kVec.y = swimPars[1];
//                    kVec.z = swimPars[2];

            }
            if (mv.surface.cylinder != null) {
                double r = 0.5 * (mv.surface.cylinder.baseArc().radius() + mv.surface.cylinder.highArc().radius());
                System.out.println(" state r = " + r);
                if (useSwimmer == false) {
                    double stepSize = 5; //mm
                    int nSteps = (int) (r / stepSize);
                    double dist = 0;
                    for (int i = 1; i < nSteps; i++) {
                        dist = (double) (i * stepSize);
                        this.setHelixPars(kVec, swim);
                        ps = util.getHelixPointAtR(dist);
                        kVec.x = ps.x();
                        kVec.y = ps.y();
                        kVec.z = ps.z();
                        this.tranState(k, kVec, swim);
                    }
                    this.setHelixPars(kVec, swim);
                    ps = new Point3D(kVec.x, kVec.y, kVec.z);
                    swimPars[0] = ps.x();
                    swimPars[1] = ps.y();
                    swimPars[2] = ps.z();
                    if (swimPars == null)
                        return null;
                } else {
                    this.setTrackPars(kVec, swim);
                    swimPars = swim.SwimRho(r / units);
                    if (swimPars == null) return null;
                    for (int j = 0; j < 3; j++) {
                        swimPars[j] *= units;
                    }
                    kVec.x = swimPars[0];
                    kVec.y = swimPars[1];
                    kVec.z = swimPars[2];
                }
            }

            value[0] = swimPars[0];
            value[1] = swimPars[1];
            value[2] = swimPars[2];
            value[3] = swimPars[3];
            value[4] = swimPars[4];
            value[5] = swimPars[5];
            value[6] = this.calcPhi(kVec);
            value[7] = swimPars[6];

        }
        return value;
    }

    public void setStateVecPosAtMeasSite(int k, StateVec kVec, MeasVec mv, Swim swimmer) {

        double[] pars = this.getStateVecPosAtMeasSite(k, kVec, mv, swimmer, true);
        if (pars == null) {
            return;
        }

        kVec.x = pars[0];
        kVec.y = pars[1];
        kVec.z = pars[2];

        kVec.alpha = new B(k, kVec.x, kVec.y, kVec.z, swimmer).alpha;
        kVec.phi = pars[6];


        double Xc = X0.get(kVec.k) + (kVec.d_rho + kVec.alpha / kVec.kappa) * Math.cos(kVec.phi0);
        double Yc = Y0.get(kVec.k) + (kVec.d_rho + kVec.alpha / kVec.kappa) * Math.sin(kVec.phi0);

        double phi_f = Math.atan2(Yc - Y0.get(k), Xc - X0.get(k));

        if (kVec.kappa < 0) {
            phi_f = Math.atan2(-Yc + Y0.get(k), -Xc + X0.get(k));
        }

        if (phi_f < 0) {
            phi_f += 2 * Math.PI;
        }
        double fphi0 = phi_f;
        double fd_rho = (Xc - X0.get(k)) * Math.cos(phi_f) + (Yc - Y0.get(k)) * Math.sin(phi_f) - kVec.alpha / kVec.kappa;
        //fkappa = iVec.kappa;
        double fdz = Z0.get(kVec.k) - Z0.get(k) + kVec.dz - (kVec.alpha / kVec.kappa) * (phi_f - kVec.phi0) * kVec.tanL;
        //ftanL = iVec.tanL;

        if (fphi0 < 0) {
            fphi0 += 2. * Math.PI;
        }

        kVec.phi0 = fphi0;
        kVec.d_rho = fd_rho;
        kVec.dz = fdz;

        kVec.pathLenght = pars[7];

    }

    public StateVec newStateVecAtMeasSite(int k, StateVec kVec, MeasVec mv, Swim swimmer, boolean useSwimmer) {

        StateVec newVec = kVec;
        double[] pars = this.getStateVecPosAtMeasSite(k, kVec, mv, swimmer, useSwimmer);
        if (pars == null) {
            return null;
        }

        newVec.x = pars[0];
        newVec.y = pars[1];
        newVec.z = pars[2];

        newVec.alpha = new B(k, newVec.x, newVec.y, newVec.z, swimmer).alpha;
        newVec.phi = pars[3];

        // new state:
        return newVec;
    }

    private void tranState(int f, StateVec iVec, Swim swimmer) {

        Bf.swimmer = swimmer;
        Bf.x = iVec.x;
        Bf.y = iVec.y;
        Bf.z = iVec.z;
        Bf.set();

        double Xc = X0.get(iVec.k) + (iVec.d_rho + Bf.alpha / iVec.kappa) * Math.cos(iVec.phi0);
        double Yc = Y0.get(iVec.k) + (iVec.d_rho + Bf.alpha / iVec.kappa) * Math.sin(iVec.phi0);

        double phi_f = Math.atan2(Yc - Y0.get(f), Xc - X0.get(f));
        if (iVec.kappa < 0) {
            phi_f = Math.atan2(-Yc + Y0.get(f), -Xc + X0.get(f));
        }

        if (phi_f < 0) {
            phi_f += 2 * Math.PI;
        }
        double fphi0 = phi_f;
        double fd_rho = (Xc - X0.get(f)) * Math.cos(phi_f) + (Yc - Y0.get(f)) * Math.sin(phi_f) - Bf.alpha / iVec.kappa;
        //fkappa = iVec.kappa;
        double fdz = Z0.get(iVec.k) - Z0.get(f) + iVec.dz - (Bf.alpha / iVec.kappa) * (phi_f - iVec.phi0) * iVec.tanL;
        //ftanL = iVec.tanL;
        double falpha = Bf.alpha;

        if (fphi0 < 0) {
            fphi0 += 2. * Math.PI;
        }

        iVec.phi0 = fphi0;
        iVec.d_rho = fd_rho;
        iVec.dz = fdz;
        iVec.alpha = falpha;

    }

    public StateVec transported(int i, int f, StateVec iVec, MeasVec mv, Swim swimmer, int dir) {

        // transport stateVec...
        StateVec fVec = new StateVec(f);

        if (iVec.phi0 < 0) {
            iVec.phi0 += 2. * Math.PI;
        }
        double x = X0.get(0) + iVec.d_rho * Math.cos(iVec.phi0);
        double y = Y0.get(0) + iVec.d_rho * Math.sin(iVec.phi0);
        double z = Z0.get(0) + iVec.dz;
        B Bf = new B(i, x, y, z, swimmer);

        fVec.phi0 = iVec.phi0;
        fVec.d_rho = iVec.d_rho;
        fVec.kappa = iVec.kappa;
        fVec.dz = iVec.dz;
        fVec.tanL = iVec.tanL;
        fVec.alpha = Bf.alpha;

        this.setStateVecPosAtMeasSite(f, fVec, mv, swimmer);

        if (dir > 0) {
            // Add energy loss here with the bethe equation found at
            // https://pdg.lbl.gov/2020/reviews/rpp2020-rev-passage-particles-matter.pdf

            double pathLenghtTotal = fVec.pathLenght - iVec.pathLenght; // in cm

            if (i == 0) {
                double rStart = Math.hypot(iVec.x, iVec.y); // mm
                double rEnd = Math.hypot(fVec.x, fVec.y); // mm
                double rTarget = 3; // mm radius of the target
                double rWallTarget = 0.06; // mm thinkness of the wall of the target
                double pathLenghtInTarget = rTarget / (rEnd - rStart) * pathLenghtTotal;
                double pathLenghtInWallTarget = rWallTarget / (rEnd - rStart) * pathLenghtTotal;
                double pathLenghtInGas = pathLenghtTotal - pathLenghtInTarget - pathLenghtInWallTarget;

                Vector3D P = this.P(i);
                double p = P.mag(); // particle momentum MeV
                double mass = 938.27208816; // proton mass in MeV/c
                double beta = p / Math.sqrt(p * p + mass * mass);
                double m_ec2 = 0.5109989461; // MeV mass of electron times c^2
                double K_ = 0.307075; // constant in Mev mol-1 cm2
                double z_p = 1; // charge number of the proton
                double gamma = 1 / (Math.sqrt(1 - beta * beta));
                double W_max = 2 * m_ec2 * beta * beta * gamma * gamma;
                double X = Math.log10(beta * gamma);

                // Energy loss in the target
                double deltaETarget = EnergyLossInTarget(pathLenghtInTarget, beta, gamma, W_max);

                // Energy loss in the Kapton (Wall of the target)
                double deltaEWall = NISTEnergyLossInKapton(pathLenghtInWallTarget);

                // Energy loss in the helium (Gas mixture)
                double deltaEHelium = EnergyLossInHelium(pathLenghtInGas, X, beta, gamma, W_max);
                double NISTdeltaEHElium = NISTEnergyLossInHelium(pathLenghtInGas);

                // Energy loss in the CO2 (Gas mixture)
                double deltaECO2 = EnergyLossInCO2(pathLenghtInGas, X, beta, gamma, W_max);
                double NISTdeltaECO2 = NISTEnergyLossInCO2(pathLenghtInGas);

                double w = 4 * 4.002602 + 44.01;
                double w1 = 4 * 4.002602 / w; // weight fraction of the 1th element (helium)
                double w2 = 44.01 / w; // weight fraction of the 2th element (CO2)
                double deltaE = w1 * deltaEHelium + w2 * deltaECO2;
                double NISTdeltaE = w1 * NISTdeltaEHElium + w2 * NISTdeltaECO2;


                fVec.Eloss = deltaETarget - deltaEWall + deltaE;
                fVec.NIST_Eloss = -deltaETarget + deltaEWall + NISTdeltaE;
            }
            else {
                //Compute the momentum at k :
                Vector3D P = this.P(i);
                double p = P.mag(); // particle momentum MeV
                double mass = 938.27208816; // proton mass in MeV/c
                double beta = p / Math.sqrt(p * p + mass * mass);
                double m_ec2 = 0.5109989461; // MeV mass of electron times c^2
                double K_ = 0.307075; // constant in Mev mol-1 cm2
                double z_p = 1; // charge number of the proton
                double gamma = 1 / (Math.sqrt(1 - beta * beta));
                double W_max = 2 * m_ec2 * beta * beta * gamma * gamma;
                double X = Math.log10(beta * gamma);

                // Energy loss in the helium (Gas mixture)
                double deltaEHelium = EnergyLossInHelium(pathLenghtTotal, X, beta, gamma, W_max);
                double NISTdeltaEHElium = NISTEnergyLossInHelium(pathLenghtTotal);

                // Energy loss in the CO2 (Gas mixture)
                double deltaECO2 = EnergyLossInCO2(pathLenghtTotal, X, beta, gamma, W_max);
                double NISTdeltaECO2 = NISTEnergyLossInCO2(pathLenghtTotal);

                double w = 4 * 4.002602 + 44.01;
                double w1 = 4 * 4.002602 / w; // weight fraction of the 1th element (helium)
                double w2 = 44.01 / w; // weight fraction of the 2th element (CO2)
                double deltaE = w1 * deltaEHelium + w2 * deltaECO2;
                double NISTdeltaE = w1 * NISTdeltaEHElium + w2 * NISTdeltaECO2;


                fVec.Eloss = deltaE;
                fVec.NIST_Eloss = NISTdeltaE;

            }
        }

        return fVec;
    }

    private double EnergyLossInTarget(double pathLenght, double beta, double gamma, double W_max){
        double K_ = 0.307075; // constant in Mev mol-1 cm2
        double m_ec2 = 0.5109989461; // MeV mass of electron times c^2
        double z_p = 1; // charge number of the proton
        double deltaX = 0; // TODO : need to be change
        double Z = 1;
        double I = 10 * Z;
        double rho = 5 * 101325 * 2.014 * 0.001 / (8.314462 * 293) * 0.001; // Ideal gas law for deuterium gas (not di deuterium) at 293°K and 7 atm
        double A = 2.014; // g/mol
       return K_ * z_p * z_p * Z / A * 1 / (beta * beta) *
                (0.5 * Math.log((2 * m_ec2 * beta * beta * gamma * gamma * W_max) / (I * I))
                        - beta * beta - deltaX / 2) * pathLenght * rho;
    }

    private double NISTEnergyLossInCO2(double pathLenght) {
        double rho = 1.842 * 0.001; // density of CO2 at 20°C and 1 atm in g/cm3 found on https://www.engineeringtoolbox.com/gas-density-d_158.html
        return 69.28 * rho * pathLenght; // for a kinematic energy of 5Mev for a proton found on https://physics.nist.gov/PhysRefData/Star/Text/PSTAR.html
    }

    private double EnergyLossInCO2(double pathLenght, double X, double beta, double gamma, double W_max){
        double K_ = 0.307075; // constant in Mev mol-1 cm2
        double m_ec2 = 0.5109989461; // MeV mass of electron times c^2
        double z_p = 1; // charge number of the proton
        double X0 = 1.629; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double X1 = 4.0; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double a = 0.1944; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double m = 3.027; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double Cbar = 10.154; // Find in https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double A = 44.01; // g/mol
        double Z = 22; // atomic number of CO2
        double rho = 1.842 * 0.001; // density of CO2 at 20°C and 1 atm in g/cm3 found on https://www.engineeringtoolbox.com/gas-density-d_158.html
        double I = 10 * Z;
        double deltaX = 0;
        if (X > X0 && X < X1) {
            deltaX = 2 * Math.log(10) * X - Cbar + a * Math.pow((X1 - X), m);
        } else if (X > X1) {
            deltaX = 2 * Math.log(10) * X - Cbar;
        } else if (X < X0) {
            deltaX = 0.;
        } // For non conductor
        // else if(X<X0) {delta0*Math.pow(10,2*(X-X0));} // for conductor
        return K_ * z_p * z_p * Z / A * 1 / (beta * beta) *
                (0.5 * Math.log((2 * m_ec2 * beta * beta * gamma * gamma * W_max) / (I * I)) -
                        beta * beta - deltaX / 2) * pathLenght * rho;
    }

    private double EnergyLossInHelium(double pathLenght, double X, double beta, double gamma, double W_max){
        double K_ = 0.307075; // constant in Mev mol-1 cm2
        double m_ec2 = 0.5109989461; // MeV mass of electron times c^2
        double z_p = 1; // charge number of the proton
        double X0 = 2.202; // Find on https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double X1 = 4.0; // Find on https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double a = 0.0114; // Find on https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double m = 7.625; // Find on https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double Cbar = 11.139; // Find on https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double Z = 2; // atomic number of helium
        double I = 10 * Z; // eV found on https://journals.aps.org/prb/abstract/10.1103/PhysRevB.26.6067
        double A = 4.002602; // g/mol
        double rho = 0.1664 * 0.001; // density of helium at 20°C and 1 atm in g/cm3 found on https://www.engineeringtoolbox.com/gas-density-d_158.html
        double deltaX = 0;
        if (X > X0 && X < X1) {
            deltaX = 2 * Math.log(10) * X - Cbar + a * Math.pow((X1 - X), m);
        }
        else if (X > X1) {
            deltaX = 2 * Math.log(10) * X - Cbar;
        }
        else if (X < X0) {
            deltaX = 0.;
        } // For non conductor
        // else if(X<X0) {delta0*Math.pow(10,2*(X-X0));} // for conductor

        return K_ * z_p * z_p * Z / A * 1 / (beta * beta) *
                (0.5 * Math.log((2 * m_ec2 * beta * beta * gamma * gamma * W_max) / (I * I))
                        - beta * beta - deltaX / 2) * pathLenght * rho;
    }

    private double NISTEnergyLossInHelium(double pathLenght){
        double rho = 0.1664 * 0.001; // density of helium at 20°C and 1 atm in g/cm3 found on https://www.engineeringtoolbox.com/gas-density-d_158.html
        return 80.5 * rho * pathLenght; // for a kinematic energy of 5Mev for a proton found on https://physics.nist.gov/PhysRefData/Star/Text/PSTAR.html
    }

    private double NISTEnergyLossInKapton(double pathLenght){
        // For Kapton using NIST Data for proton at E_k = 5 MeV
        double rho = 1.413; // density of Kapton g/cm3 for GEANT4 found http://www.apc.univ-paris7.fr/~franco/g4doxy4.10/html/_em10_materials_8cc_source.html
        return 72.21 * rho * pathLenght;
    }

    public void transport(int i, int f, StateVec iVec, CovMat icovMat, MeasVec mv, Swim swimmer, int dir) {

        StateVec fVec = this.transported(i, f, iVec, mv, swimmer, dir);

        printlnStateVec(iVec);
        printlnStateVec(fVec);

        // now transport covMat...
        double dphi0_prm_del_drho = -1. / (fVec.d_rho + iVec.alpha / iVec.kappa) * Math.sin(fVec.phi0 - iVec.phi0);
        double dphi0_prm_del_phi0 = (iVec.d_rho + iVec.alpha / iVec.kappa) / (fVec.d_rho + iVec.alpha / iVec.kappa) * Math.cos(fVec.phi0 - iVec.phi0);
        double dphi0_prm_del_kappa = (iVec.alpha / (iVec.kappa * iVec.kappa)) / (fVec.d_rho + iVec.alpha / iVec.kappa) * Math.sin(fVec.phi0 - iVec.phi0);
        double dphi0_prm_del_dz = 0;
        double dphi0_prm_del_tanL = 0;

        double drho_prm_del_drho = Math.cos(fVec.phi0 - iVec.phi0);
        double drho_prm_del_phi0 = (iVec.d_rho + iVec.alpha / iVec.kappa) * Math.sin(fVec.phi0 - iVec.phi0);
        double drho_prm_del_kappa = (iVec.alpha / (iVec.kappa * iVec.kappa)) * (1 - Math.cos(fVec.phi0 - iVec.phi0));
        double drho_prm_del_dz = 0;
        double drho_prm_del_tanL = 0;

        double dkappa_prm_del_drho = 0;
        double dkappa_prm_del_phi0 = 0;
        double dkappa_prm_del_dkappa = 1;
        double dkappa_prm_del_dz = 0;
        double dkappa_prm_del_tanL = 0;

        double dz_prm_del_drho = ((iVec.alpha / iVec.kappa) / (fVec.d_rho + iVec.alpha / iVec.kappa)) * iVec.tanL * Math.sin(fVec.phi0 - iVec.phi0);
        double dz_prm_del_phi0 = (iVec.alpha / iVec.kappa) * iVec.tanL * (1 - Math.cos(fVec.phi0 - iVec.phi0) * (iVec.d_rho + iVec.alpha / iVec.kappa) / (fVec.d_rho + iVec.alpha / iVec.kappa));
        double dz_prm_del_kappa = (iVec.alpha / (iVec.kappa * iVec.kappa)) * iVec.tanL * (fVec.phi0 - iVec.phi0 - Math.sin(fVec.phi0 - iVec.phi0) * (iVec.alpha / iVec.kappa) / (fVec.d_rho + iVec.alpha / iVec.kappa));
        double dz_prm_del_dz = 1;
        double dz_prm_del_tanL = -iVec.alpha * (fVec.phi0 - iVec.phi0) / iVec.kappa;

        double dtanL_prm_del_drho = 0;
        double dtanL_prm_del_phi0 = 0;
        double dtanL_prm_del_dkappa = 0;
        double dtanL_prm_del_dz = 0;
        double dtanL_prm_del_tanL = 1;

        double[][] FMat = new double[][]{
                {drho_prm_del_drho, drho_prm_del_phi0, drho_prm_del_kappa, drho_prm_del_dz, drho_prm_del_tanL},
                {dphi0_prm_del_drho, dphi0_prm_del_phi0, dphi0_prm_del_kappa, dphi0_prm_del_dz, dphi0_prm_del_tanL},
                {dkappa_prm_del_drho, dkappa_prm_del_phi0, dkappa_prm_del_dkappa, dkappa_prm_del_dz, dkappa_prm_del_tanL},
                {dz_prm_del_drho, dz_prm_del_phi0, dz_prm_del_kappa, dz_prm_del_dz, dz_prm_del_tanL},
                {dtanL_prm_del_drho, dtanL_prm_del_phi0, dtanL_prm_del_dkappa, dtanL_prm_del_dz, dtanL_prm_del_tanL}
        };

        //StateVec = fVec;
        this.trackTraj.put(f, fVec);
        RealMatrix F = new Array2DRowRealMatrix(FMat);
        RealMatrix FT = F.transpose();
        RealMatrix C = icovMat.covMat;
        RealMatrix Cpropagated = (FT.multiply(C)).multiply(F);
        if (Cpropagated != null) {
            CovMat fCov = new CovMat(f);
            fCov.covMat = Cpropagated.add(this.Q(iVec, fVec, f - i));
            this.trackCov.put(f, fCov);
        }
    }

    /**
     * Compute the multiple scattering noise for the covariance matrix
     *
     * @param iVec
     * @param fVec
     * @param dir
     * @return
     */
    private RealMatrix Q(StateVec iVec, StateVec fVec, int dir) {

        RealMatrix Q = MatrixUtils.createRealMatrix(5, 5);

        if (dir > 0) {
            // Formula found on Multiple scattering through small angles :
            // https://pdg.lbl.gov/2020/reviews/rpp2020-rev-passage-particles-matter.pdf


            double distance = fVec.pathLenght - iVec.pathLenght; // distance travel by the particle
            double rho = 0.8 * 0.001664 + 0.2 * 0.00184212; // gas mixture density
            double x = rho * distance;

            double pt = Math.abs(1. / iVec.kappa);
            double pz = pt * iVec.tanL;
            double p = Math.sqrt(pt * pt + pz * pz);
            double mass = 938.27208816;   // proton mass in MeV/c
            double beta = p / Math.sqrt(p * p + mass * mass); // particle momentum

            // Gas miture for ALERT : 4He-CO2
            double w_He = 4 * 4.0026 / (4 * 4.0026 + 12.0107 + 2 * 15.999); // fraction by weight of helium
            double w_C = 12.0107 / (4 * 4.0026 + 12.0107 + 2 * 15.999); // fraction by weight of carbon
            double w_O = 2 * 15.999 / (4 * 4.0026 + 12.0107 + 2 * 15.999); // fraction by weight of oxygen

            double X0_He = 94.32; // Radiation length in g/cm2 found on https://pdg.lbl.gov/2020/AtomicNuclearProperties/HTML/helium_gas_He.html
            double X0_C = 42.7; // Radiation length in g/cm2 found on https://pdg.lbl.gov/2020/AtomicNuclearProperties/HTML/carbon_amorphous_C.html
            double X0_O = 34.24; // Radiation length in g/cm2 found on https://pdg.lbl.gov/2020/AtomicNuclearProperties/HTML/oxygen_gas.html

            double X0 = 1 / (w_He / X0_He + w_C / X0_C + w_O / X0_O); // radiation length for gas mixture

            double c = 299792458; // speed of light in vacuum

            double sctRMS = 13.6 / (beta * c * p) * 1 * Math.sqrt(x / X0) * (1 + 0.038 * Math.log(x * 1 * 1 / (X0 * beta * beta))); // Lynch-Dahl formula

            double[][] q = {
                    {0, 0, 0, 0, 0},
                    {0, sctRMS * sctRMS * (1 + iVec.tanL * iVec.tanL), 0, 0, 0},
                    {0, 0, sctRMS * sctRMS * (iVec.kappa * iVec.kappa * iVec.tanL * iVec.tanL), 0, sctRMS * sctRMS * (iVec.kappa * iVec.tanL * (1 + iVec.tanL * iVec.tanL))},
                    {0, 0, 0, 0, 0},
                    {0, 0, sctRMS * sctRMS * (iVec.kappa * iVec.tanL * (1 + iVec.tanL * iVec.tanL)), 0, sctRMS * sctRMS * (1 + iVec.tanL * iVec.tanL) * (1 + iVec.tanL * iVec.tanL)}
            };

            Q = new Array2DRowRealMatrix(q);
        }

        return Q;
    }

    private StateVec reset(StateVec SV, StateVec stateVec) {
        SV = new StateVec(stateVec.k);
        SV.x = stateVec.x;
        SV.y = stateVec.y;
        SV.z = stateVec.z;
        SV.d_rho = stateVec.d_rho;
        SV.dz = stateVec.dz;
        SV.phi0 = stateVec.phi0;
        SV.phi = stateVec.phi;
        SV.tanL = stateVec.tanL;
        SV.alpha = stateVec.alpha;
        SV.kappa = stateVec.kappa;

        return SV;
    }

    private void resetArrays(double[] swimPars) {
        for (int i = 0; i < swimPars.length; i++) {
            swimPars[i] = 0;
        }
    }

    private void setHelix(Helix util, double x0, double y0, double z0, double px0, double py0, double pz0, int q, double B) {
        util.setTurningSign(q);
        util.setB(B);
        double pt = Math.sqrt(px0 * px0 + py0 * py0);
        util.setR(pt / (B * util.getLIGHTVEL()));
        util.setPhi0(Math.atan2(py0, px0));
        util.setTanL(pz0 / pt);
        util.setZ0(z0);
        util.setOmega((double) -q / util.getR());

        double S = Math.sin(util.getPhi0());
        double C = Math.cos(util.getPhi0());
        if (Math.abs(S) >= Math.abs(C)) {
            util.setD0(-(x0 - X0.get(0)) / S);
        } else {
            util.setD0((y0 - Y0.get(0)) / C);
        }

        util.Update();
    }

    private void setHelixPars(StateVec kVec, Swim swim) {
        double x0 = X0.get(kVec.k) + kVec.d_rho * Math.cos(kVec.phi0);
        double y0 = Y0.get(kVec.k) + kVec.d_rho * Math.sin(kVec.phi0);
        double z0 = Z0.get(kVec.k) + kVec.dz;
        double invKappa = 1. / Math.abs(kVec.kappa);
        double px0 = -invKappa * Math.sin(kVec.phi0);
        double py0 = invKappa * Math.cos(kVec.phi0);
        double pz0 = invKappa * kVec.tanL;

        int ch = (int) KFitter.polarity * (int) Math.signum(kVec.kappa);

        double B = 1. / (lightVel * kVec.alpha);
        this.setHelix(util, x0, y0, z0, px0, py0, pz0, ch, B);
    }

    private void setTrackPars(StateVec kVec, Swim swim) {

        double x0 = X0.get(kVec.k) + kVec.d_rho * Math.cos(kVec.phi0);
        double y0 = Y0.get(kVec.k) + kVec.d_rho * Math.sin(kVec.phi0);
        double z0 = Z0.get(kVec.k) + kVec.dz;
        double invKappa = 1. / Math.abs(kVec.kappa);
        double px0 = -invKappa * Math.sin(kVec.phi0);
        double py0 = invKappa * Math.cos(kVec.phi0);
        double pz0 = invKappa * kVec.tanL;
        int ch = (int) KFitter.polarity * (int) Math.signum(kVec.kappa);

        swim.SetSwimParameters(
                x0 / units,
                y0 / units,
                z0 / units,
                px0, py0, pz0, ch);
    }

    private double calcPhi(StateVec kVec) {
        double xc = X0.get(kVec.k) + (kVec.d_rho + kVec.alpha / kVec.kappa) * Math.cos(kVec.phi0);
        double yc = Y0.get(kVec.k) + (kVec.d_rho + kVec.alpha / kVec.kappa) * Math.sin(kVec.phi0);
        double r = Math.abs(kVec.alpha / kVec.kappa);
        Vector3D ToPoint = new Vector3D();
        Vector3D ToRef = new Vector3D(X0.get(kVec.k) - xc, Y0.get(kVec.k) - yc, 0);

        ToPoint = new Vector3D(kVec.x - xc, kVec.y - yc, 0);
        double phi = ToRef.angle(ToPoint);
        phi *= -Math.signum(kVec.kappa);

        return phi;
    }

    public static class StateVec {

        final int k;

        public double x;
        public double y;
        public double z;
        public double kappa;
        public double d_rho;
        public double phi0;
        public double phi;
        public double tanL;
        public double dz;
        public double alpha;
        public double Eloss = 0;
        public double NIST_Eloss = 0;
        public double pathLenght = 0;

        StateVec(int k) {
            this.k = k;
        }

    }

    public static class CovMat {

        final int k;
        public RealMatrix covMat;

        CovMat(int k) {
            this.k = k;
        }

    }

    public class B {

        final int k;
        public double x;
        public double y;
        public double z;
        public Swim swimmer;

        public double Bx;
        public double By;
        public double Bz;

        public double alpha;

        float b[] = new float[3];

        B(int k) {
            this.k = k;
        }

        B(int k, double x, double y, double z, Swim swimmer) {
            this.k = k;
            this.x = x;
            this.y = y;
            this.z = z;

            swimmer.BfieldLab(x / units, y / units, z / units + shift / units, b);
            this.Bx = b[0];
            this.By = b[1];
            this.Bz = b[2];

            this.alpha = 1. / (lightVel * Math.abs(b[2]));
        }

        public void set() {
            swimmer.BfieldLab(x / units, y / units, z / units + shift / units, b);
            this.Bx = b[0];
            this.By = b[1];
            this.Bz = b[2];

            this.alpha = 1. / (lightVel * Math.abs(b[2]));
        }
    }

    public double MassHypothesis(int H) {
        double piMass = 0.13957018;
        double KMass = 0.493677;
        double muMass = 0.105658369;
        double eMass = 0.000510998;
        double pMass = 0.938272029;
        double value = piMass; //default
        if (H == 4) {
            value = pMass;
        }
        if (H == 1) {
            value = eMass;
        }
        if (H == 2) {
            value = piMass;
        }
        if (H == 3) {
            value = KMass;
        }
        if (H == 0) {
            value = muMass;
        }
        return value;
    }

    /**
     * Compute the momentum with a state vector
     *
     * @param kf site k
     * @return Vector3D
     */
    public Vector3D P(int kf) {
        if (this.trackTraj.get(kf) != null) {
            double px = -(Math.signum(this.trackTraj.get(kf).kappa) / this.trackTraj.get(kf).kappa) * Math.sin(this.trackTraj.get(kf).phi0 + this.trackTraj.get(kf).phi);
            double py = (Math.signum(this.trackTraj.get(kf).kappa) / this.trackTraj.get(kf).kappa) * Math.cos(this.trackTraj.get(kf).phi0 + this.trackTraj.get(kf).phi);
            double pz = (Math.signum(this.trackTraj.get(kf).kappa) / this.trackTraj.get(kf).kappa) * this.trackTraj.get(kf).tanL;

            return new Vector3D(px, py, pz);
        } else {
            return new Vector3D(0, 0, 0);
        }

    }

    /**
     * Compute the position with a state vector
     *
     * @param kf site k
     * @return Vector3D
     */
    public Vector3D X(int kf) {
        if (this.trackTraj.get(kf) != null) {
            double x = X0.get(kf) + this.trackTraj.get(kf).d_rho * Math.cos(this.trackTraj.get(kf).phi0) + this.trackTraj.get(kf).alpha / this.trackTraj.get(kf).kappa * (Math.cos(this.trackTraj.get(kf).phi0) - Math.cos(this.trackTraj.get(kf).phi0 + this.trackTraj.get(kf).phi));
            double y = Y0.get(kf) + this.trackTraj.get(kf).d_rho * Math.sin(this.trackTraj.get(kf).phi0) + this.trackTraj.get(kf).alpha / this.trackTraj.get(kf).kappa * (Math.sin(this.trackTraj.get(kf).phi0) - Math.sin(this.trackTraj.get(kf).phi0 + this.trackTraj.get(kf).phi));
            double z = Z0.get(kf) + this.trackTraj.get(kf).dz - this.trackTraj.get(kf).alpha / this.trackTraj.get(kf).kappa * this.trackTraj.get(kf).tanL * this.trackTraj.get(kf).phi;

            return new Vector3D(x, y, z);
        } else {
            return new Vector3D(0, 0, 0);
        }

    }

    /**
     * Compute the position with a state vector and a angle phi
     *
     * @param kVec State Vector
     * @param phi  Angle
     * @return Vector3D
     */
    public Vector3D X(StateVec kVec, double phi) {
        if (kVec != null) {
            double x = X0.get(kVec.k) + kVec.d_rho * Math.cos(kVec.phi0) + kVec.alpha / kVec.kappa * (Math.cos(kVec.phi0) - Math.cos(kVec.phi0 + phi));
            double y = Y0.get(kVec.k) + kVec.d_rho * Math.sin(kVec.phi0) + kVec.alpha / kVec.kappa * (Math.sin(kVec.phi0) - Math.sin(kVec.phi0 + phi));
            double z = Z0.get(kVec.k) + kVec.dz - kVec.alpha / kVec.kappa * kVec.tanL * phi;

            return new Vector3D(x, y, z);
        } else {
            return new Vector3D(0, 0, 0);
        }

    }

    /**
     * Compute the momentum along the beamline
     *
     * @param kf site
     * @return Vector3D
     */
    public Vector3D P0(int kf) {
        if (this.trackTraj.get(kf) != null) {
            double px = -(Math.signum(this.trackTraj.get(kf).kappa) / this.trackTraj.get(kf).kappa) * Math.sin(this.trackTraj.get(kf).phi0);
            double py = (Math.signum(this.trackTraj.get(kf).kappa) / this.trackTraj.get(kf).kappa) * Math.cos(this.trackTraj.get(kf).phi0);
            double pz = (Math.signum(this.trackTraj.get(kf).kappa) / this.trackTraj.get(kf).kappa) * this.trackTraj.get(kf).tanL;

            return new Vector3D(px, py, pz);
        } else {
            return new Vector3D(0, 0, 0);
        }

    }

    /**
     * Compute the position along the beamline
     *
     * @param kf site
     * @return Vector3D
     */
    public Vector3D X0(int kf) {
        if (this.trackTraj.get(kf) != null) {
            double x = X0.get(kf) + this.trackTraj.get(kf).d_rho * Math.cos(this.trackTraj.get(kf).phi0);
            double y = Y0.get(kf) + this.trackTraj.get(kf).d_rho * Math.sin(this.trackTraj.get(kf).phi0);
            double z = Z0.get(kf) + this.trackTraj.get(kf).dz;

            return new Vector3D(x, y, z);
        } else {
            return new Vector3D(0, 0, 0);
        }

    }

    /**
     * @return Helix base on the first state vector
     */
    public Helix setTrackPars() {
        Vector3D X = this.X0(0);
        Vector3D P = this.P0(0);

        int q = KFitter.polarity * (int) Math.signum(this.trackTraj.get(0).kappa);
        double B = 1. / Math.abs(this.trackTraj.get(0).alpha) / lightVel;

        return new Helix(X.x(), X.y(), X.z(), P.x(), P.y(), P.z(), q, B, X0.get(0), Y0.get(0), util.units);
    }

    /**
     * Initialise the first state vector with the helix given by the global fit
     *
     * @param trk     Helix given by the global fit
     * @param cov     Covariance matrix (cf KFitter constructor)
     * @param swimmer cf KFitter constructor
     */
    public void init(Helix trk, Matrix cov, Swim swimmer) {
        this.units = trk.getUnitScale();
        this.lightVel = trk.getLIGHTVEL();
        this.util = trk;
        initSV.x = -trk.getD0() * Math.sin(trk.getPhi0());
        initSV.y = trk.getD0() * Math.cos(trk.getPhi0());
        initSV.z = trk.getZ0();

        double xcen = (1. / trk.getOmega() - trk.getD0()) * Math.sin(trk.getPhi0());
        double ycen = (-1. / trk.getOmega() + trk.getD0()) * Math.cos(trk.getPhi0());

        B Bf = new B(0, (float) initSV.x, (float) initSV.x, (float) initSV.z, swimmer);

        if (Math.abs(Bf.Bz) < 0.001)
            this.straight = true;

        initSV.alpha = Bf.alpha;
        initSV.kappa = Bf.alpha * trk.getOmega();
        initSV.phi0 = Math.atan2(ycen, xcen);
        if (initSV.kappa < 0) {
            initSV.phi0 = Math.atan2(-ycen, -xcen);
        }

        initSV.dz = trk.getZ();
        initSV.tanL = trk.getTanL();
        initSV.d_rho = trk.getD0() * (Math.cos(trk.getPhi0()) * Math.sin(initSV.phi0) - Math.sin(trk.getPhi0()) * Math.cos(initSV.phi0));

        double x0 = X0.get(0) + initSV.d_rho * Math.cos(initSV.phi0);
        double y0 = Y0.get(0) + initSV.d_rho * Math.sin(initSV.phi0);
        double z0 = Z0.get(0) + initSV.dz;
        double invKappa = 1. / Math.abs(initSV.kappa);
        double px0 = -invKappa * Math.sin(initSV.phi0);
        double py0 = invKappa * Math.cos(initSV.phi0);
        double pz0 = invKappa * initSV.tanL;

        initSV.phi = 0;

        this.trackTraj.put(0, initSV);
        CovMat initCM = new CovMat(0);
        cov.set(2, 2, cov.get(2, 2) * 600);
        double[][] CovKF = cov.getArray();
        initCM.covMat = new Array2DRowRealMatrix(CovKF);
        this.trackCov.put(0, initCM);
    }

    /**
     * prints the matrix -- used for debugging
     *
     * @param C matrix
     */
    public void printMatrix(Matrix C) {
        for (int k = 0; k < 5; k++) {
            System.out.println(C.get(k, 0) + "	" + C.get(k, 1) + "	" + C.get(k, 2) + "	" + C.get(k, 3) + "	" + C.get(k, 4));
        }
    }

    /**
     * print the state vector -- use for debugging
     *
     * @param S State vector
     */
    public void printlnStateVec(StateVec S) {
        System.out.println(S.k + ") drho " + S.d_rho + " phi0 " + S.phi0 + " kappa " + S.kappa + " dz " + S.dz + " tanL " + S.tanL + " phi " + S.phi + " r = " + Math.hypot(S.x, S.y) + " x " + S.x + " y " + S.y + " z " + S.z + " alpha " + S.alpha);
    }
}
