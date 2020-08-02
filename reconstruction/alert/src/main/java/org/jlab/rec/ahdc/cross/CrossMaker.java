package org.jlab.rec.ahdc.cross;

import org.jlab.geom.prim.Point3D;
import org.jlab.geom.prim.Vector3D;
import org.jlab.rec.ahdc.cluster.Cluster;

import java.util.ArrayList;

public class CrossMaker {


    public CrossMaker() {
    }

    private ArrayList<Cross> crosses = new ArrayList<Cross>();

    public void findCross(ArrayList<Cluster> allclusters){
        for(Cluster clus : allclusters){
            double x = clus.get_R()*Math.cos(clus.get_Phi());
            double y = clus.get_R()*Math.sin(clus.get_Phi());
            double z = clus.get_Z();
            Cross cross = new Cross(new Point3D(x,y,z), new Vector3D());
            crosses.add(cross);
        }

    }

    public ArrayList<Cross> getCrosses() {
        return crosses;
    }

    public void setCrosses(ArrayList<Cross> crosses) {
        this.crosses = crosses;
    }
}

