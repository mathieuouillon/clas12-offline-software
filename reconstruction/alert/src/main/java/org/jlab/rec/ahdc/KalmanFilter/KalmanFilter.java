package org.jlab.rec.ahdc.KalmanFilter;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealMatrixFormat;
import org.jlab.clas.swimtools.Swim;
import org.jlab.clas.tracking.kalmanfilter.Surface;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class KalmanFilter {

    boolean debug = false;
    boolean debug1 = false;
    boolean energyLoss = false;

    RealMatrixFormat TABLE_FORMAT = new RealMatrixFormat("", "", "", "\n", "", ", ");

    Swim swim;

    List<MeasurementPoint> measurementPoints = new ArrayList<>();

    List<RealMatrix> Fs = new ArrayList<>();

    List<ExtrapolateStateVector> extrapolateStateVectors = new ArrayList<>();
    List<RealMatrix> extrapolateCovarianceMatrices = new ArrayList<>();

    List<UpdateStateVector> updateStateVectors = new ArrayList<>();
    List<RealMatrix> updateCovarianceMatrices = new ArrayList<>();

    List<SmoothStateVector> smoothStateVectors;
    List<RealMatrix> smoothedCovarianceMatrices;

    public static class MeasurementPoint {
        double xo;
        double yo;
        double zo;

        RealMatrix m;

        public MeasurementPoint(double xo, double yo, double zo, RealMatrix m) {
            this.xo = xo;
            this.yo = yo;
            this.zo = zo;

            this.m = m;
        }

        @Override
        public String toString() {
            return "MeasurementPoint{" +
                    "xo=" + xo +
                    ", yo=" + yo +
                    ", zo=" + zo +
                    '}';
        }
    }

    public abstract static class StateVector {

        RealMatrix q;

        MeasurementPoint m;

        double alpha;

        public void computeAlpha(double x, double y, double z, Swim swim) {
            float[] b = new float[3];
            swim.BfieldLab(x / 10, y / 10, z / 10, b);
            double c = 0.000299792458; // speed of light
            this.alpha = 1. / (c * Math.sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]));
        }

        public double d_rho() {
            return q.getEntry(0, 0);
        }

        public double phi_0() {
            return q.getEntry(1, 0);
        }

        public double kappa() {
            return q.getEntry(2, 0);
        }

        public double d_z() {
            return q.getEntry(3, 0);
        }

        public double tanL() {
            return q.getEntry(4, 0);
        }

        public void set_kappa(double kappa) {
            q.setEntry(2, 0, kappa);
        }

    }

    public static class ExtrapolateStateVector extends StateVector {

        double x;
        double y;
        double z;

        double path_length;

        public ExtrapolateStateVector(RealMatrix q, MeasurementPoint m, double x, double y, double z, double path_length, Swim swim) {

            this.q = q;

            this.m = m;

            this.x = x;
            this.y = y;
            this.z = z;

            this.path_length = path_length;

            computeAlpha(x, y, z, swim);

        }


    }

    public static class UpdateStateVector extends StateVector {

        public UpdateStateVector(RealMatrix q, MeasurementPoint m, Swim swim) {
            this.q = q;

            this.m = m;

            double x = m.xo + this.d_rho() * Math.cos(this.phi_0());
            double y = m.yo + this.d_rho() * Math.sin(this.phi_0());
            double z = m.zo + this.d_z();

            computeAlpha(x, y, z, swim);
        }
    }

    public static class SmoothStateVector extends StateVector {

        public SmoothStateVector(RealMatrix q, MeasurementPoint m, Swim swim) {
            this.q = q;

            this.m = m;

            double x = m.xo + this.d_rho() * Math.cos(this.phi_0());
            double y = m.yo + this.d_rho() * Math.sin(this.phi_0());
            double z = m.zo + this.d_z();

            computeAlpha(x, y, z, swim);
        }
    }

    public KalmanFilter(Swim swimmer) {
        swim = swimmer;
    }

    public void runKalmanFilter(double[] positionAndMomentumFromHelixFit, List<Surface> measSurfaces, double p_gf, double phi_gf, double theta_gf) {

        initializationMeasurementList(measSurfaces);

        initializationStateVectorAndCovarianceMatrix(positionAndMomentumFromHelixFit);

        double measurement_error = 0.1;

        int numberOfIteration = 3;

        for (int iteration = 0; iteration < numberOfIteration; iteration++) {

            if (debug) {
                System.out.println("Extrapolation and Filtering : ");
                printUpdateStateVector(updateStateVectors.get(0));
            }

            for (int k = 0; k < measurementPoints.size() - 1; k++) {

                if (debug) System.out.println("k = " + k);

                if (k == 0) {
                    UpdateStateVector q_hat = updateStateVectors.get(k);
                    RealMatrix C = updateCovarianceMatrices.get(k);
                    MeasurementPoint m = q_hat.m;

                    // Extrapolation part :
                    ExtrapolateStateVector q_hat_minus = f(q_hat, m, 3.0000);

                    // Energy loss :
                    if (energyLoss) energyLossInDeuterium(q_hat_minus);

                    RealMatrix F = F(q_hat, q_hat_minus);
                    RealMatrix Q = Q(q_hat, q_hat_minus, 0);
                    RealMatrix C_minus = (F.multiply(C).multiply(F.transpose())).add(Q);

                    extrapolateStateVectors.add(q_hat_minus);
                    extrapolateCovarianceMatrices.add(C_minus);
                    Fs.add(F);

                    updateStateVectors.add(new UpdateStateVector(q_hat_minus.q, q_hat_minus.m, swim));
                    updateCovarianceMatrices.add(C_minus);

                    if (debug) printUpdateStateVector(new UpdateStateVector(q_hat_minus.q, q_hat_minus.m, swim));

                    q_hat = updateStateVectors.get(k + 1);
                    C = updateCovarianceMatrices.get(k+1);
                    m = q_hat.m;

                    // Extrapolation part :
                    q_hat_minus = f(q_hat, m, 3.0600);

                    // Energy loss :
                    if (energyLoss) energyLossInKapton(q_hat_minus);

                    F = F(q_hat, q_hat_minus);
                    Q = Q(q_hat, q_hat_minus, 1);
                    C_minus = (F.multiply(C).multiply(F.transpose())).add(Q);

                    extrapolateStateVectors.add(q_hat_minus);
                    extrapolateCovarianceMatrices.add(C_minus);
                    Fs.add(F);

                    updateStateVectors.add(new UpdateStateVector(q_hat_minus.q, q_hat_minus.m, swim));
                    updateCovarianceMatrices.add(C_minus);

                    if (debug) printUpdateStateVector(new UpdateStateVector(q_hat_minus.q, q_hat_minus.m, swim));
                }


                UpdateStateVector q_hat = updateStateVectors.get(k + 2);
                RealMatrix C = updateCovarianceMatrices.get(k + 2);
                MeasurementPoint m = q_hat.m;
                MeasurementPoint m_prim = measurementPoints.get(k + 1);

                // Extrapolation part :
                ExtrapolateStateVector q_hat_minus = f(q_hat, m, m_prim);

                // Energy loss :
                if (energyLoss) energyLossInGasMixture(q_hat_minus);

                RealMatrix F = F(q_hat, q_hat_minus);
                RealMatrix Q = Q(q_hat, q_hat_minus, 2);
                RealMatrix C_minus = (F.multiply(C).multiply(F.transpose())).add(Q);

                extrapolateStateVectors.add(q_hat_minus);
                extrapolateCovarianceMatrices.add(C_minus);
                Fs.add(F);

                // Update part :
                RealMatrix h = h(q_hat_minus);
                RealMatrix H = H(q_hat, m, m_prim);
                RealMatrix y = m_prim.m.subtract(h);
                RealMatrix R = MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(measurement_error);
                RealMatrix S = (H.multiply(C_minus).multiply(H.transpose())).add(R);
                RealMatrix K = C_minus.multiply(H.transpose()).multiply(MatrixUtils.inverse(S));
                RealMatrix I = MatrixUtils.createRealIdentityMatrix(5);

                RealMatrix q = q_hat_minus.q.add(K.multiply(y));
                C = (I.subtract(K.multiply(H))).multiply(C_minus);


                updateStateVectors.add(new UpdateStateVector(q, m_prim, swim));
                updateCovarianceMatrices.add(C);

                if (debug) printUpdateStateVector(new UpdateStateVector(q, m_prim, swim));

            }


            // RTS Smoother :
            if (debug) System.out.println("Smoother start : ");

            // Create list for smoothed state vectors and covariance matrix
            int size = updateStateVectors.size();
            smoothStateVectors = new ArrayList<>(Collections.nCopies(size, null));
            smoothedCovarianceMatrices = new ArrayList<>(Collections.nCopies(size, null));

            // last update state vector become last smooth state vector
            int last = updateStateVectors.size() - 1;
            SmoothStateVector q_s_last = new SmoothStateVector(updateStateVectors.get(last).q, updateStateVectors.get(last).m, swim);
            smoothStateVectors.set(last, q_s_last);
            smoothedCovarianceMatrices.set(last, updateCovarianceMatrices.get(last));

            if (debug) printSmoothStateVector(q_s_last);

            for (int k = smoothStateVectors.size() - 1; k > 0; k--) {

                if (debug) System.out.println("k = " + k);

                RealMatrix F = Fs.get(k);
                RealMatrix C = updateCovarianceMatrices.get(k - 1);
                RealMatrix C_minus = extrapolateCovarianceMatrices.get(k);
                RealMatrix C_s = smoothedCovarianceMatrices.get(k);
                RealMatrix q_hat = updateStateVectors.get(k - 1).q;
                RealMatrix q_hat_s = smoothStateVectors.get(k).q;
                RealMatrix q_hat_minus = extrapolateStateVectors.get(k).q;

                RealMatrix A = C.multiply(F.transpose()).multiply(MatrixUtils.inverse(C_minus)); // A_{k-1} = C_{k-1|k-1} F^T C_{k|k-1}^-1

                q_hat_s = q_hat.add(A.multiply(q_hat_s.subtract(q_hat_minus))); // q^s_{k-1} = q_{k-1|k-1} + A_{k-1} (q^s_{k} - q_{k|k-1})

                C_s = C.subtract(A.multiply(C_minus.subtract(C_s)).multiply(A.transpose())); // P_{k-1}^s = P_{k-1|k-1} - A_{k-1} (P_{k|k-1} - P_{k}^s) A^T_{k-1}

                smoothStateVectors.set(k - 1, new SmoothStateVector(q_hat_s, updateStateVectors.get(k - 1).m, swim));
                smoothedCovarianceMatrices.set(k - 1, C_s);

                if (debug)
                    printSmoothStateVector(new SmoothStateVector(q_hat_s, updateStateVectors.get(k - 1).m, swim));

            }

            extrapolateStateVectors.clear();
            extrapolateCovarianceMatrices.clear();
            updateStateVectors.clear();
            extrapolateCovarianceMatrices.clear();
            Fs.clear();

            extrapolateStateVectors.add(null);
            extrapolateCovarianceMatrices.add(null);
            Fs.add(null);
            UpdateStateVector first = new UpdateStateVector(smoothStateVectors.get(0).q, measurementPoints.get(0), swim);
            updateStateVectors.add(first);
            updateCovarianceMatrices.add(smoothedCovarianceMatrices.get(0));
        }


        output(p_gf, phi_gf, theta_gf);


    }

    private void initializationMeasurementList(List<Surface> measSurfaces) {
        for (Surface meas : measSurfaces) {
            RealMatrix m_Matrix = MatrixUtils.createRealMatrix(3, 1);
            m_Matrix.setColumn(0, new double[]{meas.refPoint.x(), meas.refPoint.y(), meas.refPoint.z()});
            MeasurementPoint m = new MeasurementPoint(meas.refPoint.x(), meas.refPoint.y(), meas.refPoint.z(), m_Matrix);
            measurementPoints.add(m);
        }
    }

    private void initializationStateVectorAndCovarianceMatrix(double[] starting) {

        extrapolateStateVectors.add(null);
        extrapolateCovarianceMatrices.add(null);
        Fs.add(null);

        double x = starting[0];
        double y = starting[1];
        double z = starting[2];
        double px = starting[3];
        double py = starting[4];
        double pz = starting[5];

        double kappa = 1. / Math.sqrt(px * px + py * py);
        double d_rho = 1e-3;
        double phi_0 = Math.atan2(-px, py);
        double tanL = pz * kappa;
        double d_z = 1e-3;

        RealMatrix q = MatrixUtils.createRealMatrix(5, 1);
        q.setColumn(0, new double[]{d_rho, phi_0, kappa, d_z, tanL});

        UpdateStateVector q_hat = new UpdateStateVector(q, measurementPoints.get(0), swim);

        updateStateVectors.add(q_hat);

        /*
        RealMatrix C = MatrixUtils.createRealMatrix(5, 5);
        C.setEntry(0, 0, d_rho * d_rho);
        C.setEntry(1, 0, d_rho * phi_0);
        C.setEntry(0, 1, d_rho * phi_0);
        C.setEntry(2, 0, d_rho * kappa);
        C.setEntry(0, 2, d_rho * kappa);
        C.setEntry(2, 1, phi_0 * kappa);
        C.setEntry(1, 2, phi_0 * kappa);
        C.setEntry(1, 1, phi_0 * phi_0);
        C.setEntry(2, 2, kappa * kappa);
        C.setEntry(3, 3, d_z * d_z);
        C.setEntry(3, 4, tanL * d_z);
        C.setEntry(4, 3, tanL * d_z);
        C.setEntry(4, 4, tanL * tanL);
        */

        RealMatrix C = MatrixUtils.createRealIdentityMatrix(5).scalarMultiply(0.00000000001);
        updateCovarianceMatrices.add(C);


    }

    private void output(double p_gf, double phi_gf, double theta_gf) {

        double kappa = smoothStateVectors.get(0).kappa();
        double phi_0 = smoothStateVectors.get(0).phi_0();
        double tanL = smoothStateVectors.get(0).tanL();
        double px = -1. / Math.abs(kappa) * Math.sin(phi_0);
        double py = 1. / Math.abs(kappa) * Math.cos(phi_0);
        double pz = 1. / Math.abs(kappa) * tanL;
        double p = Math.sqrt(px * px + py * py + pz * pz);

        double phi_kf = phi_0 + Math.PI / 2;
        if (phi_kf > Math.PI) phi_kf -= 2 * Math.PI;
        if (phi_kf < -Math.PI) phi_kf += 2 * Math.PI;
        phi_kf = Math.toDegrees(phi_kf);

        double theta_kf = Math.toDegrees(Math.PI - Math.atan(tanL) - Math.PI / 2);

        if (debug) System.out.println("p_gf = " + p_gf);
        if (debug) System.out.println("p_kf = " + p * 1000);
        if (debug) System.out.println("phi_gf = " + phi_gf);
        if (debug) System.out.println("phi_kf = " + phi_kf);
        if (debug) System.out.println("theta_gf = " + theta_gf);
        if (debug) System.out.println("theta_kf = " + theta_kf + '\n');

        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter("Geant4Info.txt", true));
            writer.write("mom_gf = " + p_gf + '\n');
            writer.write("mom_kf = " + p + '\n');
            writer.write("phi_gf = " + phi_gf + '\n');
            writer.write("phi_kf = " + phi_kf + '\n');
            writer.write("theta_gf = " + theta_gf + '\n');
            writer.write("theta_kf = " + theta_kf + '\n');
            writer.write('\n');
            writer.close();
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

    }

    private ExtrapolateStateVector f(UpdateStateVector q, MeasurementPoint m, MeasurementPoint m_prim) {

        double accuracy = 1e-6;
        double stepSize = 1e-5;

        double xo = m.xo + q.d_rho() * Math.cos(q.phi_0());
        double yo = m.yo + q.d_rho() * Math.sin(q.phi_0());
        double zo = m.zo + q.d_z();
        double pxo = -1. / Math.abs(q.kappa()) * Math.sin(q.phi_0());
        double pyo = 1. / Math.abs(q.kappa()) * Math.cos(q.phi_0());
        double pzo = 1. / Math.abs(q.kappa()) * q.tanL();

        if (debug1)
            System.out.println("Swim from : x = " + xo + " y = " + yo + " z = " + zo + " px = " + pxo + " pyo = " + pyo + " pzo = " + pzo);

        double xo_prim = m_prim.xo;
        double yo_prim = m_prim.yo;
        double zo_prim = m_prim.zo;

        double r = Math.hypot(xo_prim, yo_prim);

        if (debug1) System.out.println("expected r = " + r);

        swim.SetSwimParameters(xo / 10, yo / 10, zo / 10, pxo, pyo, pzo, 1, stepSize, accuracy);

        double[] swimParams = swim.SwimRho(r / 10);

        if (debug1) System.out.println("Swim To : " + Arrays.toString(swimParams));

        try {
            double x = swimParams[0] * 10;
            double y = swimParams[1] * 10;
            double z = swimParams[2] * 10;
            double px = swimParams[3];
            double py = swimParams[4];
            double pz = swimParams[5];

            if (debug1) System.out.println(" reached r = " + Math.hypot(x, y));

            double kappa = 1. / Math.sqrt(px * px + py * py);
            double d_rho = Math.sqrt((x - xo_prim) * (x - xo_prim) + (y - yo_prim) * (y - yo_prim));
            double phi_0 = Math.atan2(-px, py);
            double tanL = pz * kappa;
            double d_z = z - zo_prim;

            RealMatrix q_prim = MatrixUtils.createRealMatrix(5, 1);
            q_prim.setColumn(0, new double[]{d_rho, phi_0, kappa, d_z, tanL});
            return new ExtrapolateStateVector(q_prim, m_prim, x, y, z, swimParams[6], swim);

        } catch (NullPointerException e) {
            throw new NullPointerException("swimParams is null");
        }


    }

    private ExtrapolateStateVector f(UpdateStateVector q, MeasurementPoint m, double r) {

        double accuracy = 1e-10;
        double stepSize = 1e-8;

        double xo = m.xo + q.d_rho() * Math.cos(q.phi_0());
        double yo = m.yo + q.d_rho() * Math.sin(q.phi_0());
        double zo = m.zo + q.d_z();
        double pxo = -1. / Math.abs(q.kappa()) * Math.sin(q.phi_0());
        double pyo = 1. / Math.abs(q.kappa()) * Math.cos(q.phi_0());
        double pzo = 1. / Math.abs(q.kappa()) * q.tanL();

        if (debug1)
            System.out.println("Swim from : x = " + xo + " y = " + yo + " z = " + zo + " px = " + pxo + " pyo = " + pyo + " pzo = " + pzo);

        if (debug1) System.out.println("expected r = " + r);

        swim.SetSwimParameters(xo / 10, yo / 10, zo / 10, pxo, pyo, pzo, 1, stepSize, accuracy);

        double[] swimParams = swim.SwimRho(r / 10);

        if (debug1) System.out.println("Swim To : " + Arrays.toString(swimParams));

        try {
            double x = swimParams[0] * 10;
            double y = swimParams[1] * 10;
            double z = swimParams[2] * 10;
            double px = swimParams[3];
            double py = swimParams[4];
            double pz = swimParams[5];

            if (debug1) System.out.println(" reached r = " + Math.hypot(x, y));

            double kappa = 1. / Math.sqrt(px * px + py * py);
            double d_rho = 1e-2;
            double phi_0 = Math.atan2(-px, py);
            double tanL = pz * kappa;
            double d_z = 1e-2;

            RealMatrix q_prim = MatrixUtils.createRealMatrix(5, 1);
            q_prim.setColumn(0, new double[]{d_rho, phi_0, kappa, d_z, tanL});
            MeasurementPoint m_prim = new MeasurementPoint(x, y, z, MatrixUtils.createRealMatrix(new double[][]{{x}, {y}, {z}}));
            return new ExtrapolateStateVector(q_prim, m_prim, x, y, z, swimParams[6], swim);

        } catch (NullPointerException e) {
            throw new NullPointerException("swimParams is null");
        }


    }

    private RealMatrix F(UpdateStateVector q, ExtrapolateStateVector q_prim) {

        double d_rho = q.d_rho();
        double phi_0 = q.phi_0();
        double kappa = q.kappa();
        double d_z = q.d_z();
        double tanL = q.tanL();

        double fd_rho = q_prim.d_rho();
        double fphi_0 = q_prim.phi_0();
        double fkappa = q_prim.kappa();
        double fd_z = q_prim.d_z();
        double ftanL = q_prim.tanL();

        double alpha = q.alpha;


        double dphi0_prm_del_drho = -1. / (fd_rho + alpha / kappa) * Math.sin(fphi_0 - phi_0);
        double dphi0_prm_del_phi0 = (d_rho + alpha / kappa) / (fd_rho + alpha / kappa) * Math.cos(fphi_0 - phi_0);
        double dphi0_prm_del_kappa = (alpha / (kappa * kappa)) / (fd_rho + alpha / kappa) * Math.sin(fphi_0 - phi_0);
        double dphi0_prm_del_dz = 0;
        double dphi0_prm_del_tanL = 0;

        double drho_prm_del_drho = Math.cos(fphi_0 - phi_0);
        double drho_prm_del_phi0 = (d_rho + alpha / kappa) * Math.sin(fphi_0 - phi_0);
        double drho_prm_del_kappa = (alpha / (kappa * kappa)) * (1 - Math.cos(fphi_0 - phi_0));
        double drho_prm_del_dz = 0;
        double drho_prm_del_tanL = 0;

        double dkappa_prm_del_drho = 0;
        double dkappa_prm_del_phi0 = 0;
        double dkappa_prm_del_dkappa = 1;
        double dkappa_prm_del_dz = 0;
        double dkappa_prm_del_tanL = 0;

        double dz_prm_del_drho = ((alpha / kappa) / (fd_rho + alpha / kappa)) * tanL * Math.sin(fphi_0 - phi_0);
        double dz_prm_del_phi0 = (alpha / kappa) * tanL * (1 - Math.cos(fphi_0 - phi_0) * (fd_rho + (alpha / kappa)) / (fd_rho + (alpha / kappa)));
        double dz_prm_del_kappa = (alpha / (kappa * kappa)) * tanL * (fphi_0 - phi_0 - Math.sin(fphi_0 - phi_0) * (alpha / kappa) / (fd_rho + alpha / kappa));
        double dz_prm_del_dz = 1;
        double dz_prm_del_tanL = -alpha * (fphi_0 - phi_0) / kappa;

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

        return MatrixUtils.createRealMatrix(FMat);

    }

    private RealMatrix Q(UpdateStateVector q, ExtrapolateStateVector q_prim, int k) {
        double pt = Math.abs(1. / q.kappa());
        double pz = pt * q.tanL();
        double p = Math.sqrt(pt * pt + pz * pz);
        double mass = 0.93827208816;   // proton mass in GeV/c^2
        double beta = p / Math.sqrt(p * p + mass * mass); // particle momentum

        // Gas miture for ALERT : 4He-CO2
        double w_He = 4 * 4.0026 / (4 * 4.0026 + 12.0107 + 2 * 15.999); // fraction by weight of helium
        double w_C = 12.0107 / (4 * 4.0026 + 12.0107 + 2 * 15.999); // fraction by weight of carbon
        double w_O = 2 * 15.999 / (4 * 4.0026 + 12.0107 + 2 * 15.999); // fraction by weight of oxygen

        double X0_He = 94.32; // Radiation length in g/cm2 found on https://pdg.lbl.gov/2020/AtomicNuclearProperties/HTML/helium_gas_He.html
        double X0_C = 42.7; // Radiation length in g/cm2 found on https://pdg.lbl.gov/2020/AtomicNuclearProperties/HTML/carbon_amorphous_C.html
        double X0_O = 34.24; // Radiation length in g/cm2 found on https://pdg.lbl.gov/2020/AtomicNuclearProperties/HTML/oxygen_gas.html

        double X0;
        if (k == 0) {
            X0 = 125.98; // radiation length for Deuterium
        } else if (k == 1) {
            X0 = 40.58; // radiation length for Kapton
        } else {
            X0 = 1 / (w_He / X0_He + w_C / X0_C + w_O / X0_O); // radiation length for gas mixture
        }

        double pathLenght = q_prim.path_length;

        double sctRMS = 0.0141 / (beta * p) * Math.sqrt(pathLenght / X0) * (1 + 1. / 9. * Math.log10(pathLenght / X0)); // Lynch-Dahl formula

        double[][] Q_Mat = {
                {0, 0, 0, 0, 0},
                {0, sctRMS * sctRMS * (1 + q.tanL() * q.tanL()), 0, 0, 0},
                {0, 0, sctRMS * sctRMS * (q.kappa() * q.kappa() * q.tanL()
                        * q.tanL()), 0, sctRMS * sctRMS * (q.kappa() * q.tanL()
                        * (1 + q.tanL() * q.tanL()))},
                {0, 0, 0, 0, 0},
                {0, 0, sctRMS * sctRMS * (q.kappa() * q.tanL() * (1 + q.tanL()
                        * q.tanL())), 0, sctRMS * sctRMS * (1 + q.tanL() * q.tanL())
                        * (1 + q.tanL() * q.tanL())}};

        RealMatrix Q = MatrixUtils.createRealMatrix(Q_Mat);
        RealMatrix F = F(q, q_prim);
        return F.multiply(Q).multiply(F.transpose());

    }

    private RealMatrix h(ExtrapolateStateVector q) {
        RealMatrix h = MatrixUtils.createRealMatrix(3, 1);
        h.setColumn(0, new double[]{q.x, q.y, q.z});
        return h;
    }

    private RealMatrix H(UpdateStateVector q, MeasurementPoint m, MeasurementPoint m_prim) {

        RealMatrix H = MatrixUtils.createRealMatrix(3, 5);

        double sqrt_epsilon = Math.sqrt(Math.ulp(1f));

        for (int i = 0; i < 5; i++) {
            UpdateStateVector qPlus = new UpdateStateVector(q.q.copy(), m, swim);
            UpdateStateVector qMinus = new UpdateStateVector(q.q.copy(), m, swim);

            double h = sqrt_epsilon * (q.q.getEntry(i, 0));

            qPlus.q.setEntry(i, 0, q.q.getEntry(i, 0) + h);
            qMinus.q.setEntry(i, 0, q.q.getEntry(i, 0) - h);

            ExtrapolateStateVector qPlus_prim = f(qPlus, m, m_prim);
            ExtrapolateStateVector qMinus_prim = f(qMinus, m, m_prim);

            //Energy loss :
            if (energyLoss) energyLossInGasMixture(qPlus_prim);
            if (energyLoss) energyLossInGasMixture(qMinus_prim);

            double dx_di = (qPlus_prim.x - qMinus_prim.x) / (2 * h);
            double dy_di = (qPlus_prim.y - qMinus_prim.y) / (2 * h);
            double dz_di = (qPlus_prim.z - qMinus_prim.z) / (2 * h);
            H.setColumn(i, new double[]{dx_di, dy_di, dz_di});
        }

        return H;
    }

    private void energyLossInKapton(ExtrapolateStateVector q) {

        double path_length = q.path_length;

        double DeltaE = -1.187E+02 * 1.42000E+00 * path_length / 1000; // in GeV

        double pt = Math.abs(1. / q.kappa()); // GeV
        double pz = pt * q.tanL(); // GeV
        double p = Math.sqrt(pt * pt + pz * pz); // Gev

        double mass = 0.93827208816;

        double E = Math.sqrt(p * p + mass * mass);

        double kappa_prim = q.kappa() * (p) / (Math.sqrt(p * p + 2 * E * DeltaE + DeltaE * DeltaE));

        if (debug) System.out.println("path_length = " + path_length);
        if (debug) System.out.println("DeltaE = " + DeltaE);
        if (debug) System.out.println("kappa = " + q.kappa());
        if (debug) System.out.println("kappa_prim = " + kappa_prim);

        q.set_kappa(kappa_prim);

    }

    private void energyLossInDeuterium(ExtrapolateStateVector q) {

        double path_length = q.path_length;

        double M_D2 = 2.014101764;
        double rhoHelium = 5 * 101325 * M_D2 / (8.31446261815324 * 293.15) * 1e-6;
        double DeltaE = -3.136E+02 * rhoHelium * path_length / 1000;

        double pt = Math.abs(1. / q.kappa()); // GeV
        double pz = pt * q.tanL(); // GeV
        double p = Math.sqrt(pt * pt + pz * pz); // Gev

        double mass = 0.93827208816;

        double E = Math.sqrt(p * p + mass * mass);

        double kappa_prim = q.kappa() * (p) / (Math.sqrt(p * p + 2 * E * DeltaE + DeltaE * DeltaE));

        if (debug) System.out.println("kappa = " + q.kappa());
        if (debug) System.out.println("kappa_prim = " + kappa_prim);

        q.set_kappa(kappa_prim);
    }

    private void energyLossInGasMixture(ExtrapolateStateVector q) {

        double path_length = q.path_length;

        double Mavg = 0.2 * 44.01 + 0.8 * 4.0026;
        double rhoGasMixture = 101325 * Mavg / (8.31446261815324 * 293.15) * 1e-6;

        double A_He = 4.002602; // atomic mass g mol-1
        double A_C = 12.0107; // g mol-1 found on https://pdg.lbl.gov/2020/AtomicNuclearProperties/HTML/carbon_amorphous_C.html
        double A_O = 15.999; // g mol-1 found on https://pdg.lbl.gov/2020/AtomicNuclearProperties/HTML/oxygen_gas.html

        double w_He = 0.8 * A_He / (0.8 * A_He + 0.2 * (A_C + 2 * A_O)); // weight fraction of the 1th element (helium)
        double w_CO2 = 0.2 * (A_C + 2 * A_O) / (0.8 * A_He + 0.2 * (A_C + 2 * A_O)); // weight fraction of the 2th element (CO2)

        double DeltaE = -(w_CO2 * 1.136E+02 + w_He * 1.360E+02) * rhoGasMixture * path_length / 1000; //GeV g cm-2

        double pt = Math.abs(1. / q.kappa()); // GeV
        double pz = pt * q.tanL(); // GeV
        double p = Math.sqrt(pt * pt + pz * pz); // Gev

        double mass = 0.93827208816;

        double E = Math.sqrt(p * p + mass * mass);

        double kappa_prim = q.kappa() * (p) / (Math.sqrt(p * p + 2 * E * DeltaE + DeltaE * DeltaE));

        q.set_kappa(kappa_prim);

    }

    private void printExtrapolateStateVector(ExtrapolateStateVector q) {
        double xo = q.m.xo + q.d_rho() * Math.cos(q.phi_0());
        double yo = q.m.yo + q.d_rho() * Math.sin(q.phi_0());
        double zo = q.m.zo + q.d_z();
        double pxo = -1. / Math.abs(q.kappa()) * Math.sin(q.phi_0());
        double pyo = 1. / Math.abs(q.kappa()) * Math.cos(q.phi_0());
        double pzo = 1. / Math.abs(q.kappa()) * q.tanL();

        System.out.println("Extrapolate State Vector : x = " + xo + " y = " + yo + " z = " + zo + " px = " + pxo + " py = " + pyo + " pz = " + pzo + " x swim = " + q.x + " y swim = " + q.y + " z swim = " + q.z);
        System.out.println("Measurement Point : x = " + q.m.xo + " y = " + q.m.yo + " z = " + q.m.zo);
        System.out.println("distance = " + Math.sqrt((xo - q.m.xo) * (xo - q.m.xo) + (yo - q.m.yo) * (yo - q.m.yo) + (zo - q.m.zo) * (zo - q.m.zo)) + '\n');
    }

    private void printUpdateStateVector(UpdateStateVector q) {
        double xo = q.m.xo + q.d_rho() * Math.cos(q.phi_0());
        double yo = q.m.yo + q.d_rho() * Math.sin(q.phi_0());
        double zo = q.m.zo + q.d_z();
        double pxo = -1. / Math.abs(q.kappa()) * Math.sin(q.phi_0());
        double pyo = 1. / Math.abs(q.kappa()) * Math.cos(q.phi_0());
        double pzo = 1. / Math.abs(q.kappa()) * q.tanL();

        double p = Math.sqrt(pxo * pxo + pyo * pyo + pzo * pzo);

        System.out.println("drho = " + q.d_rho() + " phi0 = " + q.phi_0() + " kappa = " + q.kappa() + " dz = " + q.d_z() + " tanL = " + q.tanL());
        System.out.println("Update State Vector : x = " + xo + " y = " + yo + " z = " + zo + " px = " + pxo + " py = " + pyo + " pz = " + pzo + " p = " + p);
        System.out.println("Measurement Point : x = " + q.m.xo + " y = " + q.m.yo + " z = " + q.m.zo);
        System.out.println("distance = " + Math.sqrt((xo - q.m.xo) * (xo - q.m.xo) + (yo - q.m.yo) * (yo - q.m.yo) + (zo - q.m.zo) * (zo - q.m.zo)) + '\n');
    }

    private void printSmoothStateVector(SmoothStateVector q) {
        double xo = q.m.xo + q.d_rho() * Math.cos(q.phi_0());
        double yo = q.m.yo + q.d_rho() * Math.sin(q.phi_0());
        double zo = q.m.zo + q.d_z();
        double pxo = -1. / Math.abs(q.kappa()) * Math.sin(q.phi_0());
        double pyo = 1. / Math.abs(q.kappa()) * Math.cos(q.phi_0());
        double pzo = 1. / Math.abs(q.kappa()) * q.tanL();

        double p = Math.sqrt(pxo * pxo + pyo * pyo + pzo * pzo);

        System.out.println("d_rho = " + q.d_rho() + " phi_0 = " + q.phi_0() + " kappa = " + q.kappa() + " d_z = " + q.d_z() + " tanL = " + q.tanL());
        System.out.println("Smooth State Vector : x = " + xo + " y = " + yo + " z = " + zo + " px = " + pxo + " py = " + pyo + " pz = " + pzo + " p = " + p);
        System.out.println("Measurement Point : x = " + q.m.xo + " y = " + q.m.yo + " z = " + q.m.zo);
        System.out.println("distance = " + Math.sqrt((xo - q.m.xo) * (xo - q.m.xo) + (yo - q.m.yo) * (yo - q.m.yo) + (zo - q.m.zo) * (zo - q.m.zo)) + '\n');

    }

}
