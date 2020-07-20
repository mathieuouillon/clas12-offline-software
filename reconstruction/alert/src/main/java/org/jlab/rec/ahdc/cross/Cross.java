package org.jlab.rec.ahdc.cross;

import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;

public class Cross {

    private Point3D _Point;
    private Vector3D _Dir;

    public Cross(Point3D _Point, Vector3D _Dir) {
        this._Point = _Point;
        this._Dir = _Dir;
    }

}
