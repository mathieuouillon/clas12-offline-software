package org.jlab.rec.rtpc.RTPCKalmanFilter;

import Jama.Matrix;
import org.jlab.clas.swimtools.Swim;
import org.jlab.clas.tracking.trackrep.Helix;
import org.jlab.geom.prim.Vector3D;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.rtpc.hit.FinalTrackInfo;
import org.jlab.rec.rtpc.hit.HitParameters;
import org.jlab.rec.rtpc.hit.RecoHitVector;

import java.util.HashMap;
import java.util.List;

public class KalmanFilter {

    private final RecUtilities recUtil = new RecUtilities();



    public KalmanFilter(HitParameters params) {
        this.init(params);
    }

    /**
     * Transform global fit parameters R, A, B, Z_0 and Theta into state vector parameters d_rho, phi_0,
     * kappa, d_z, tanL
     * @param params
     */
    private void init(HitParameters params) {
        HashMap<Integer, FinalTrackInfo> finaltrackinfomap = params.get_finaltrackinfomap();
        HashMap<Integer, List<RecoHitVector>> recotrackmap = params.get_recotrackmap();
        for (int TID : recotrackmap.keySet()) {
            double xc = finaltrackinfomap.get(TID).get_A(); // x coordinates of the center of helix
            double yc = finaltrackinfomap.get(TID).get_B(); // y coordinates of the center of helix
            double x = recotrackmap.get(TID).get(0).x(); // x coordinates of the first hit of track
            double y = recotrackmap.get(TID).get(0).y(); // y coordinates of the first hit of track
            double d_rho = Math.abs(Math.sqrt((x - xc)*(x - xc) + (y - yc)*(y - yc)) - finaltrackinfomap.get(TID).get_R());
            double m1 = (y- yc)/(x - xc);
            double m2 = 0; // = (yc - yc)/(xc + 1 -xc)
            double phi_0 = Math.abs(Math.atan((m1-m2)/(1+m1*m2))); // Angle between two lines based on slope
            double kappa = 1 / finaltrackinfomap.get(TID).get_R();
            double d_z = finaltrackinfomap.get(TID).get_vz();
            double tanL = Math.tan(-Math.toRadians(finaltrackinfomap.get(TID).get_theta()) + Math.PI / 2);

            // System.out.println("d_rho = " + d_rho + " phi_0 = " + phi_0 + " kappa = " + kappa + " d_z = " + d_z + " tanL = " + tanL);

            Matrix cov = new Matrix(5, 5, 0.0);
            cov.set(0, 0, d_rho * d_rho);
            cov.set(1, 0, phi_0 * d_rho);
            cov.set(2, 0, kappa * d_rho);
            cov.set(0, 1, d_rho * phi_0);
            cov.set(0, 2, d_rho * kappa);
            cov.set(1, 1, phi_0 * phi_0);
            cov.set(1, 2, phi_0 * kappa);
            cov.set(2, 1, phi_0 * kappa);
            cov.set(2, 2, kappa * kappa);
            cov.set(3, 3, d_z * d_z);
            cov.set(3, 4, d_z * tanL);
            cov.set(4, 3, d_z * tanL);
            cov.set(4, 4, tanL * tanL);

            finaltrackinfomap.get(TID).set_d_rho(d_rho);
            finaltrackinfomap.get(TID).set_phi_0(phi_0);
            finaltrackinfomap.get(TID).set_kappa(kappa);
            finaltrackinfomap.get(TID).set_d_z(d_z);
            finaltrackinfomap.get(TID).set_tanL(tanL);
            finaltrackinfomap.get(TID).set_cov(cov);

        }
    }

    public void process(HitParameters params, DataEvent event){
        Swim swimmer = new Swim();
        org.jlab.clas.tracking.kalmanfilter.helical.KFitter kf = null;
        HashMap<Integer, FinalTrackInfo> finaltrackinfomap = params.get_finaltrackinfomap();
        HashMap<Integer, List<RecoHitVector>> recotrackmap = params.get_recotrackmap();

        for (int TID : recotrackmap.keySet()) {

            double xr = finaltrackinfomap.get(TID).get_d_rho() * Math.sin(finaltrackinfomap.get(TID).get_phi_0());
            double yr = finaltrackinfomap.get(TID).get_d_rho() * Math.cos(finaltrackinfomap.get(TID).get_phi_0());
            double zr = finaltrackinfomap.get(TID).get_d_z();
            double pt = Constants.LIGHTVEL * 1 / finaltrackinfomap.get(TID).get_kappa() * Constants.getSolenoidVal();
            double pz = pt * finaltrackinfomap.get(TID).get_tanL();
            double px = pt * Math.cos(finaltrackinfomap.get(TID).get_phi_0());
            double py = pt * Math.sin(finaltrackinfomap.get(TID).get_phi_0());

            int charge = (int) (Math.signum(Constants.getSolenoidscale()) * Constants.CHARGE);

            xr += Constants.Xb;
            yr += Constants.Yb;

            Helix hlx = new Helix(xr, yr, zr, px, py, pz, charge, Constants.getSolenoidVal(), Constants.Xb, Constants.Yb, Helix.Units.MM);


            kf = new org.jlab.clas.tracking.kalmanfilter.helical.KFitter(hlx, finaltrackinfomap.get(TID).get_cov(),
                    event, swimmer, Constants.Xb, Constants.Yb, Constants.Zoffset,
                    recUtil.setMeasVecs(recotrackmap.get(TID)));
            kf.runFitter(swimmer);

            finaltrackinfomap.get(TID).setKFHelix(kf.KFHelix);

/*
            System.out.println("pt : " + pt);
            System.out.println("pt hlx : " + Math.sqrt(finaltrackinfomap.get(TID).get_px()/1000*finaltrackinfomap.get(TID).get_px()/1000 + finaltrackinfomap.get(TID).get_py()/1000*finaltrackinfomap.get(TID).get_py()/1000));
            System.out.println("pt kf : " + kf.KFHelix.getPx()*kf.KFHelix.getPx() + kf.KFHelix.getPy()*kf.KFHelix.getPy());

            System.out.println("p : " + Math.sqrt(px*px + py*py + pz*pz));
            System.out.println("p hlx : " + Math.sqrt(finaltrackinfomap.get(TID).get_px()/1000*finaltrackinfomap.get(TID).get_px()/1000 + finaltrackinfomap.get(TID).get_py()/1000*finaltrackinfomap.get(TID).get_py()/1000
                    + finaltrackinfomap.get(TID).get_py()/1000*finaltrackinfomap.get(TID).get_py()/1000));
            System.out.println("p kf : " + kf.KFHelix.getPx()*kf.KFHelix.getPx() + kf.KFHelix.getPy()*kf.KFHelix.getPy() +kf.KFHelix.getPz()*kf.KFHelix.getPz());
*/

        }
    }
}
