package org.jlab.rec.rtpc.RTPCKalmanFilter;

public class Constants {
    public static final double LIGHTVEL = 0.000299792458;       // velocity of light (mm/ns) - conversion factor from radius in mm to momentum in GeV/c

    public static final double Xb = 0;
    public static final double Yb = 0;

    public static final double Zoffset = 0;

    public static final int CHARGE = 1;

    private static double SOLENOIDSCALE = 1.;
    private static double SOLENOIDVAL = 5.;

    public static final void setSolenoidscale(double scale) {
        SOLENOIDSCALE = scale;
    }

    public static final double getSolenoidscale() {
        return SOLENOIDSCALE;
    }

    public static final void setSolenoidVal(double val) {
        SOLENOIDVAL = val;
    }

    public static final double getSolenoidVal() {
        return SOLENOIDVAL;
    }
}
