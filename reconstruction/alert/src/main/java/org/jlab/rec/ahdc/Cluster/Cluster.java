package org.jlab.rec.ahdc.Cluster;

import org.jlab.rec.ahdc.Hit.Hit;
import java.util.ArrayList;

/**
 * Cluster is list of hit that are grouped together
 * according to the algorithm of the ClusterFinder class
 */


public class Cluster extends ArrayList<Hit> {
    private double _R;
    private double _Phi;
    private double _Z;


    public Cluster(double _R, double _Phi, double _Z) {
        this._R = _R;
        this._Phi = _Phi;
        this._Z = _Z;

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
