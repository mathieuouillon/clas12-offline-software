package org.jlab.rec.ahdc.Cross;

import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;
import org.jlab.rec.ahdc.Cluster.Cluster;

import java.util.ArrayList;

public class CrossMaker {


    public CrossMaker() {
    }

    private ArrayList<Cross> crosses = new ArrayList<Cross>();

    public void findCross(ArrayList<Cluster> ahdc_cluster){
        for(Cluster cluster : ahdc_cluster){
            double X = - cluster.get_R() * Math.sin(cluster.get_Phi()) ;
            double Y = - cluster.get_R() * Math.cos(cluster.get_Phi());
            double Z = cluster.get_Z();
            crosses.add(new Cross(X,Y,Z));
        }
    }

    public ArrayList<Cross> get_Crosses() {
        return crosses;
    }

    public void set_Crosses(ArrayList<Cross> crosses) {
        this.crosses = crosses;
    }
}

