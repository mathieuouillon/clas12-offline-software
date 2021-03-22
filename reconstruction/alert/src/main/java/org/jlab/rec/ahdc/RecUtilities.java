package org.jlab.rec.ahdc;

import org.jlab.clas.swimtools.Swim;
import org.jlab.clas.tracking.kalmanfilter.Surface;
import org.jlab.clas.tracking.objects.Strip;
import org.jlab.geom.prim.Arc3D;
import org.jlab.geom.prim.Cylindrical3D;
import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;
import org.jlab.rec.ahdc.Hit.Hit;
import org.jlab.rec.ahdc.Hit.TrueHit;


import java.awt.*;
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
    // For real data
    public List<Surface> setMeasVecs(List<Hit> AHDC_Hits) {
        List<Surface> KFSites = new ArrayList<Surface>();
        int id = 0;
        for(Hit hit : AHDC_Hits){
            double z_start = -150;
            double z_end = 150;
            double x_start = hit.get_X();
            double y_start = hit.get_Y();
            double x_end = hit.get_Radius() * Math.sin(hit.get_Phi()+Math.toRadians(20));
            double y_end = hit.get_Radius() * Math.cos(hit.get_Phi()+Math.toRadians(20));
            Point3D origin = new Point3D(x_start,y_start,z_start);
            Point3D center = new Point3D(0,0,z_start);
            Vector3D normal = new Vector3D(0,0,1);
            double theta = 2*Math.PI;
            Arc3D arc = new Arc3D(origin,center,normal,theta);
            double height = Math.sqrt(x_start*x_start + y_start*y_start);
            Cylindrical3D cyl = new Cylindrical3D(arc,height);
            Strip strip = new Strip(id,0,x_start,y_start,z_start,x_end,y_end,z_end);
            Surface meas = new Surface(cyl,strip);
            meas.setLayer(id);
            meas.setError(0.01);
            meas.setSector(0);
            KFSites.add(meas);
            id++;
        }
        return KFSites;
    }


    //For simulated data
    public List<Surface> setMeasVecsGeant4(List<TrueHit> AHDC_Hits) {

        List<Surface> KFSites = new ArrayList<Surface>();
        int id = 0;
        for (TrueHit hit : AHDC_Hits) {
            if (hit.get_Pid() == 2212) {
                Point3D origin = new Point3D(hit.get_X(), hit.get_Y(), hit.get_Z());
                Point3D center = new Point3D(0, 0, hit.get_Z());
                Vector3D normal = new Vector3D(0, 0, 1);
                double theta = 2 * Math.PI;
                Arc3D arc = new Arc3D(origin, center, normal, theta);
                double height = Math.hypot(hit.get_X(), hit.get_Y());
                Cylindrical3D cyl = new Cylindrical3D(arc, height);
                Surface meas = new Surface(cyl, origin);
                meas.setLayer(id);
                meas.setError(1e-6);
                meas.setSector(0);
                KFSites.add(meas);
                id++;
            }
        }
        return KFSites;
    }
    
}