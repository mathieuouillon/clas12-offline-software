package org.jlab.rec.ahdc.PreCluster;

public class PreCluster {

    private int _Id;
    private int _Superlayer;
    private int _Layer;
    private int _Wire;
    private double _Doca;
    private double _Radius;
    private double _Phi_0;

    public PreCluster(int _Superlayer, int _Layer, int _Wire, double _Doca, double _Radius, double _Phi_0){
        this._Superlayer = _Superlayer;
        this._Layer = _Layer;
        this._Wire = _Wire;
        this._Doca = _Doca;
        this._Radius = _Radius;
        this._Phi_0 = _Phi_0;
    }

    public String toString(){
        return "PreCluster : Superlayer : " + this._Superlayer + " Layer : " + this._Layer + " Wire : " + this._Wire + " Radius : " + this._Radius + " Phi_0 : " + this._Phi_0;
    }
}
