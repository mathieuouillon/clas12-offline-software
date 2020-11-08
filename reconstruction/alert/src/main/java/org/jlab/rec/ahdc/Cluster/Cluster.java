package org.jlab.rec.ahdc.Cluster;

import org.jlab.rec.ahdc.Hit.Hit;
import org.jlab.rec.ahdc.PreCluster.PreCluster;

import java.util.ArrayList;

public class Cluster extends ArrayList<Hit> {
    private double _R;
    private double _Phi;
    private double _Z;


    public Cluster(PreCluster precluster, PreCluster otherprecluster) {
        this._R = ( precluster.get_Radius() + otherprecluster.get_Radius()) /2;
        this._Phi = (precluster.get_Phi_0() + otherprecluster.get_Phi_0())/2;
        this._Z = ((otherprecluster.get_Phi_0() - precluster.get_Phi_0())/(Math.toRadians(20)*Math.pow(-1,precluster.get_Superlayer()) - Math.toRadians(20)*Math.pow(-1,otherprecluster.get_Superlayer())))*300-150;
    }

    public double get_R() {
        return _R;
    }

    public void set_R(double _R) {
        this._R = _R;
    }

    public double get_Phi() {
        return _Phi;
    }

    public void set_Phi(double _Phi) {
        this._Phi = _Phi;
    }

    public double get_Z() {
        return _Z;
    }

    public void set_Z(double _Z) {
        this._Z = _Z;
    }
}
