package org.jlab.rec.ahdc.cross;

import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;

public class Cross {

    private Point3D _Point;
    private Vector3D _Dir;

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

    public Cross(Point3D _Point, Vector3D _Dir) {
        this._Point = _Point;
        this._Dir = _Dir;
    }

}
