package org.jlab.rec.ahdc.Hit;


public class Hit implements Comparable<Hit> {

    private int _Id;
    private int _Super_layer;
    private int _Layer;
    private int _Wire;
    private double _Doca;
    private double _Phi;
    private double _Radius;
    private int _Num_wire;
    private boolean _Used = false;
    private double _X;
    private double _Y;


    public Hit(int _Super_layer, int _Layer, int _Wire, double _Doca) {
        this._Super_layer = _Super_layer;
        this._Layer = _Layer;
        this._Wire = _Wire;
        this._Doca = _Doca;
        if(this._Super_layer == 0 && this._Layer == 0){calcRadiusAndPhi(32, 47);}
        if(this._Super_layer == 1 && this._Layer == 0){calcRadiusAndPhi(38, 56);}
        if(this._Super_layer == 1 && this._Layer == 1){calcRadiusAndPhi(42, 56);}
        if(this._Super_layer == 2 && this._Layer == 0){calcRadiusAndPhi(48, 72);}
        if(this._Super_layer == 2 && this._Layer == 1){calcRadiusAndPhi(52, 72);}
        if(this._Super_layer == 3 && this._Layer == 0){calcRadiusAndPhi(58, 87);}
        if(this._Super_layer == 3 && this._Layer == 1){calcRadiusAndPhi(62, 87);}
        if(this._Super_layer == 4 && this._Layer == 0){calcRadiusAndPhi(68, 99);}
    }

    private void calcRadiusAndPhi(double radius, int num_wire){
        this._Phi = this._Wire * ((2* Math.PI)/num_wire) - (2 * Math.PI) / num_wire;
        this._Radius = radius;
        this._Num_wire = num_wire;
        this._X = - this._Radius * Math.sin(this._Phi);
        this._Y = - this._Radius * Math.cos(this._Phi);

    }

    @Override
    public String toString() {
        return "Hit{" +
                "_Super_layer=" + _Super_layer +
                ", _Layer=" + _Layer +
                ", _Wire=" + _Wire +
                ", _Doca=" + _Doca +
                ", _Phi=" + _Phi +
                '}';
    }

    @Override
    public int compareTo(Hit arg0) {
        if ((this._Super_layer == arg0._Super_layer && this._Layer == arg0._Layer && this._Wire > arg0._Wire)
            || (this._Super_layer == arg0._Super_layer && this._Layer > arg0._Layer)
            || (this._Super_layer > arg0._Super_layer)) {
            return 1;
        }
        else {
            return 0;
        }
    }

    public int get_Id() {
        return _Id;
    }

    public void set_Id(int _id) {
        _Id = _id;
    }

    public int get_Super_layer() {
        return _Super_layer;
    }

    public void set_Super_layer(int _Super_layer) {
        this._Super_layer = _Super_layer;
    }

    public int get_Layer() {
        return _Layer;
    }

    public void set_Layer(int _Layer) {
        this._Layer = _Layer;
    }

    public int get_Wire() {
        return _Wire;
    }

    public void set_Wire(int _Wire) {
        this._Wire = _Wire;
    }

    public double get_Doca() {
        return _Doca;
    }

    public void set_Doca(double _Doca) {
        this._Doca = _Doca;
    }

    public double get_Phi() {
        return _Phi;
    }

    public void set_Phi(double _Phi) {
        this._Phi = _Phi;
    }

    public double get_Radius() {
        return _Radius;
    }

    public void set_Radius(double _Radius) {
        this._Radius = _Radius;
    }

    public int get_Num_wire() {
        return _Num_wire;
    }

    public void set_Num_wire(int _Num_wire) {
        this._Num_wire = _Num_wire;
    }

    public boolean is_Used() {
        return _Used;
    }

    public void set_Used(boolean _Used) {
        this._Used = _Used;
    }

    public double get_X() {
        return _X;
    }

    public void set_X(double _X) {
        this._X = _X;
    }

    public double get_Y() {
        return _Y;
    }

    public void set_Y(double _Y) {
        this._Y = _Y;
    }

}
