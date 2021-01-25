package org.jlab.rec.rtpc.RTPCKalmanFilter;

import org.jlab.clas.tracking.kalmanfilter.Surface;
import org.jlab.geom.prim.Arc3D;
import org.jlab.geom.prim.Cylindrical3D;
import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;
import org.jlab.rec.rtpc.hit.RecoHitVector;

import java.util.ArrayList;
import java.util.List;

/**
 * Service to return reconstructed TRACKS
 * format
 *
 * @author ziegler
 *
 */
public class RecUtilities {
    
    public List<Surface> setMeasVecs(List<RecoHitVector> recoHitVectors) {

        List<Surface> KFSites = new ArrayList<>();
        /* Point3D origin = new Point3D(0.0, 0.0, 0.0); // TODO : Un point en 0,0,z avec z un parametre libre
        Point3D center = new Point3D(0.0, 0.0, 0.0);
        Vector3D normal = new Vector3D(0.0, 0.0, 1.0);
        double theta = 2*Math.PI;
        Arc3D arc = new Arc3D(origin, center, normal, theta);
        double height = 0.0;
        Cylindrical3D cylinder = new Cylindrical3D(arc, height);
        Point3D point = new Point3D(Constants.Xb, Constants.Yb, Constants.Zoffset);
        Surface meas0 = new Surface(cylinder, point);
        meas0.setLayer(0);
        meas0.setSector(0);
        meas0.setError(0.0);
        KFSites.add(meas0); */

        for(RecoHitVector hit : recoHitVectors){
            Point3D origin = new Point3D(hit.x(), hit.y(), hit.z());
            Point3D center = new Point3D(0.0, 0.0, hit.z());
            Vector3D normal = new Vector3D(0.0, 0.0, 1.0);
            double theta = 2*Math.PI;
            Arc3D arc = new Arc3D(origin, center, normal, theta);
            double height = Math.sqrt(hit.x()*hit.x() + hit.y()*hit.y());
            Cylindrical3D cyl = new Cylindrical3D(arc, height);
            Point3D pnt = new Point3D(hit.x(), hit.y(), hit.z());
            Surface meas = new Surface(cyl, pnt);
            meas.setError(0);
            meas.setLayer(0);
            meas.setSector(0);
            KFSites.add(meas);
        }
        return KFSites;
    }
}