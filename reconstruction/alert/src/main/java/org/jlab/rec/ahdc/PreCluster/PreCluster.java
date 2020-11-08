package org.jlab.rec.ahdc.PreCluster;

public class PreCluster {

    private int _Id;
    private int _Superlayer;
    private int _Layer;
    private double _Doca;
    private double _Radius;
    private double _Phi_0;

    public PreCluster(int _Superlayer, int _Layer, double _Doca, double _Radius, double _Phi_0){
        this._Superlayer = _Superlayer;
        this._Layer = _Layer;
        this._Doca = _Doca;
        this._Radius = _Radius;
        this._Phi_0 = _Phi_0;
    }

    public String toString(){
        return "PreCluster : Superlayer : " + this._Superlayer + " Layer : " + this._Layer + " Radius : " + this._Radius + " Phi_0 : " + this._Phi_0;
    }

    public int get_Superlayer() {
        return _Superlayer;
    }

    public void set_Superlayer(int _Superlayer) {
        this._Superlayer = _Superlayer;
    }

    public int get_Layer() {
        return _Layer;
    }

    public void set_Layer(int _Layer) {
        this._Layer = _Layer;
    }

    public double get_Doca() {
        return _Doca;
    }

    public void set_Doca(double _Doca) {
        this._Doca = _Doca;
    }

    public double get_Radius() {
        return _Radius;
    }

    public void set_Radius(double _Radius) {
        this._Radius = _Radius;
    }

    public double get_Phi_0() {
        return _Phi_0;
    }

    public void set_Phi_0(double _Phi_0) {
        this._Phi_0 = _Phi_0;
    }
}
