package org.jlab.rec.ahdc.KalmanFilter;

import org.jlab.clas.swimtools.Swim;
import org.jlab.clas.tracking.trackrep.Helix;
import org.jlab.rec.ahdc.Hit.TrueHit;

import java.util.Arrays;
import java.util.List;

public class KalmanFilter {

    public KalmanFilter(List<TrueHit> measList, Helix helix, Swim swimmer){

        // Initialization of the state vector at 0, 0, z
        StateVector x = new StateVector(0);
        x.x = helix.getX();
        x.y = helix.getY();
        x.z = helix.getZ();
        x.px = helix.getPx();
        x.py = helix.getPy();
        x.pz = helix.getPz();

        System.out.println(x);

        double units = 10.;
        int ch = 1;

        swimmer.SetSwimParameters(x.x / units, x.y / units,x.z / units, x.px, x.py, x.pz, ch);

        // Extrapolation of the state vector
        int k = 0;
        double[] swimPars = new double[7];
        double r = Math.hypot(measList.get(k).get_X(), measList.get(k).get_Y()); // mm
        swimPars = swimmer.SwimRho(r/units); // en cm



        for(int j =0; j < 3; j++) {
            swimPars[j]*=units;
        }

        // Compute Kalman gain matrix


        System.out.println();
    }

    private class StateVector {
        final int k;
        public double x;
        public double y;
        public double z;
        public double px;
        public double py;
        public double pz;

        StateVector(int k) {
            this.k = k;
        }

        @Override
        public String toString() {
            return "StateVector{" +
                    "k=" + k +
                    ", x=" + x +
                    ", y=" + y +
                    ", z=" + z +
                    ", px=" + px +
                    ", py=" + py +
                    ", pz=" + pz +
                    '}';
        }
    }
}
