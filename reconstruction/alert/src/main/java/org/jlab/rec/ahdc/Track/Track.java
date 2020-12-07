package org.jlab.rec.ahdc.Track;

import org.jlab.rec.ahdc.Cluster.Cluster;

import java.util.ArrayList;
import java.util.List;

public class Track {

    private double _Distance;
    private List<Cluster> _Clusters = new ArrayList<>();
    private boolean _Used = false;

    public Track(List<Cluster> clusters){
        this._Clusters = clusters;
        this._Distance = 0;
        for(int i = 0; i < clusters.size()-1; i++){
            this._Distance += Math.sqrt( (clusters.get(i).get_X() - clusters.get(i+1).get_X())*(clusters.get(i).get_X() - clusters.get(i+1).get_X()) +
                    (clusters.get(i).get_Y() - clusters.get(i+1).get_Y())*(clusters.get(i).get_Y() - clusters.get(i+1).get_Y()) );
        }
    }

    @Override
    public String toString() {
        return "Track{" +
                "_Clusters=" + _Clusters +
                '}';
    }

    public double get_Distance() {
        return _Distance;
    }

    public void set_Distance(double _Distance) {
        this._Distance = _Distance;
    }

    public List<Cluster> get_Clusters() {
        return _Clusters;
    }

    public void set_Clusters(List<Cluster> _Clusters) {
        this._Clusters = _Clusters;
    }

    public boolean is_Used() {
        return _Used;
    }

    public void set_Used(boolean _Used) {
        this._Used = _Used;
    }
}
