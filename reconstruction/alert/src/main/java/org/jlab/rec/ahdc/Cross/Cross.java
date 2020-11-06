package org.jlab.rec.ahdc.Cross;

import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;

public class Cross {



    private double _Radius;
    private Point3D _Point;
    private Vector3D _Dir;

    public Point3D get_PointErr() {
        return _PointErr;
    }

    public void set_PointErr(Point3D _PointErr) {
        this._PointErr = _PointErr;
    }

    private Point3D _PointErr;



    private double _Phi;

    public Point3D get_Point() {
        return _Point;
    }

    public void set_Point(Point3D _Point) {
        this._Point = _Point;
    }

    public Vector3D get_Dir() {
        return _Dir;
    }

    public void set_Dir(Vector3D _Dir) {
        this._Dir = _Dir;
    }

    public double get_Radius() {
        return _Radius;
    }

    public void set_Radius(double _Radius) {
        this._Radius = _Radius;
    }

    public double get_Phi() {
        return _Phi;
    }

    public void set_Phi(double _Phi) {
        this._Phi = _Phi;
    }

    public Cross(Point3D _Point, Vector3D _Dir) {
        this._Point = _Point;
        this._Dir = _Dir;
    }


}
