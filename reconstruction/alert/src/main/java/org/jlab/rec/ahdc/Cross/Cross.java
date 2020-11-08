package org.jlab.rec.ahdc.Cross;

import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;

public class Cross {

    private double _X;
    private double _Y;

    public double get_X() {
        return _X;
    }

    public void set_X(double _X) {
        this._X = _X;
    }

    public double get_Y() {
        return _Y;
    }

    public void set_Y(double _Y) {
        this._Y = _Y;
    }

    public double get_Z() {
        return _Z;
    }

    public void set_Z(double _Z) {
        this._Z = _Z;
    }

    private double _Z;


    public Cross(double X, double Y, double Z) {
        this._X = X;
        this._Y = Y;
        this._Z = Z;
    }
}
