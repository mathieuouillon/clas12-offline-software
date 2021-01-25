package org.jlab.rec.rtpc.RTPCKalmanFilter;

import Jama.Matrix;
import org.jlab.clas.swimtools.Swim;
import org.jlab.clas.tracking.trackrep.Helix;
import org.jlab.geom.prim.Vector3D;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.rtpc.hit.FinalTrackInfo;
import org.jlab.rec.rtpc.hit.HitParameters;
import org.jlab.rec.rtpc.hit.RecoHitVector;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.StandardOpenOption;
import java.util.HashMap;
import java.util.List;

public class KalmanFilter {

    private final RecUtilities recUtil = new RecUtilities();



    public KalmanFilter(HitParameters params) {
        this.init(params);
    }

    private void init(HitParameters params) {

    }

    public void process(HitParameters params, DataEvent event){
        Swim swimmer = new Swim();
        HashMap<Integer, FinalTrackInfo> finaltrackinfomap = params.get_finaltrackinfomap();
        HashMap<Integer, List<RecoHitVector>> recotrackmap = params.get_recotrackmap();

        for (int TID : finaltrackinfomap.keySet()) {

            double B = 5.0;

            double x = finaltrackinfomap.get(TID).get_X0();
            double y = finaltrackinfomap.get(TID).get_Y0();
            double z = finaltrackinfomap.get(TID).get_Z0();
            double px =  finaltrackinfomap.get(TID).get_px()/1000;
            double py =  finaltrackinfomap.get(TID).get_py()/1000;
            double pz =  finaltrackinfomap.get(TID).get_pz()/1000;
            int q = 1;

            Helix hlx = new Helix(x, y, z, px, py, pz, q, B, Constants.Xb, Constants.Yb, Helix.Units.MM);

            double omega = hlx.getOmega();
            double tanL = hlx.getTanL();
            double phi0 = hlx.getPhi0();
            double d0 = hlx.getD0();
            double z0 = hlx.getZ0();

            Matrix cov = new Matrix(5, 5);
            cov.set(0, 0, d0 * d0);
            cov.set(1, 0, phi0 * d0);
            cov.set(0, 1, d0 * phi0);
            cov.set(2, 0, omega * d0);
            cov.set(0, 2, d0 * omega);
            cov.set(1, 1, phi0 * phi0);
            cov.set(1, 2, phi0 * omega);
            cov.set(2, 1, phi0 * omega);
            cov.set(2, 2, omega * omega);
            cov.set(3, 3, z0 * z0);
            cov.set(4, 4, tanL * tanL);

            org.jlab.clas.tracking.kalmanfilter.helical.KFitter kf = new org.jlab.clas.tracking.kalmanfilter.helical.KFitter(hlx, cov,
                    event, swimmer, Constants.Xb, Constants.Yb, Constants.Zoffset, recUtil.setMeasVecs(recotrackmap.get(TID)));

            kf.runFitter(swimmer);

            finaltrackinfomap.get(TID).setKFHelix(kf.KFHelix);

            /*double px_gf = finaltrackinfomap.get(TID).get_px()/1000;
            double py_gf = finaltrackinfomap.get(TID).get_py()/1000;
            double pz_gf = finaltrackinfomap.get(TID).get_pz()/1000;

            System.out.println(" Global fit : px = " + px_gf + " py = " + py_gf + " pz = " + pz_gf);

            double px_kf = kf.KFHelix.getPx();
            double py_kf = kf.KFHelix.getPy();
            double pz_kf = kf.KFHelix.getPz();

            System.out.println(" Kalman fit : px = " + px_kf + " py = " + py_kf + " pz = " + pz_kf);*/

            try {
                BufferedWriter writer = new BufferedWriter(new FileWriter("filename.txt", true));
                double px_gf = finaltrackinfomap.get(TID).get_px()/1000;
                double py_gf = finaltrackinfomap.get(TID).get_py()/1000;
                double pz_gf = finaltrackinfomap.get(TID).get_pz()/1000;
                writer.write("p global = " + Math.sqrt(px_gf*px_gf + py_gf*py_gf + pz_gf*pz_gf) + '\n');
                writer.write("mom global = " + 0.3*50*Math.abs(finaltrackinfomap.get(TID).get_R())/(10*Math.sin(Math.toRadians(finaltrackinfomap.get(TID).get_theta())))/1000 + '\n');
                double px_kf = kf.KFHelix.getPx();
                double py_kf = kf.KFHelix.getPy();
                double pz_kf = kf.KFHelix.getPz();
                double Theta = Math.PI/2.-Math.atan(kf.KFHelix.getTanL());
                double Phi = kf.KFHelix.getPhi0();
                if(Phi> Math.PI) Phi-=2*Math.PI;
                if(Phi<-Math.PI) Phi+=2*Math.PI;
                writer.write("p kf = " + Math.sqrt(px_kf*px_kf + py_kf*py_kf + pz_kf*pz_kf) + '\n');
                double mom = 0.000299792458*5.0*1/(kf.KFHelix.getOmega());
                writer.write("mom = " + mom + '\n');
                writer.write("theta global = " + finaltrackinfomap.get(TID).get_theta() + '\n');
                Theta = Math.toDegrees(Theta);
                writer.write("theta kf = " + Theta + '\n');
                writer.write("phi global = " + finaltrackinfomap.get(TID).get_phi() + '\n');
                double Phi_deg=Math.toDegrees(Phi);
                if(Phi_deg >= 180) Phi_deg -= 360;
                if(Phi_deg < -180) Phi_deg += 360;
                writer.write("phi kf = " + Phi_deg + '\n');
                double omega_kf = kf.KFHelix.getOmega();
                writer.write("omega global = " + omega + '\n');
                writer.write("omega kf = " + omega_kf + '\n');
                double z_kf = kf.KFHelix.getZ();;
                double z_gf = hlx.getZ();
                writer.write("z global = " + z_gf + '\n');
                writer.write("z kf = " + z_kf + '\n');
                writer.write('\n');
                writer.close();
            } catch (IOException e) {
                System.out.println("An error occurred.");
                e.printStackTrace();
            }
        }
    }

    public void printMatrix(Matrix C) {
        System.out.println("    ");
        for (int k = 0; k < 5; k++) {
            System.out.println(C.get(k, 0)+"\t"+C.get(k, 1)+"\t"+C.get(k, 2)+"\t"+C.get(k, 3)+"\t"+C.get(k, 4));
        }
        System.out.println("    ");
    }
}
