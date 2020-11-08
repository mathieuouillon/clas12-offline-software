package org.jlab.rec.ahdc.Hit;


public class Hit implements Comparable<Hit> {

    private int _Id;
    private int _Superlayer;
    private int _Layer;
    private int _Wire;
    private double _Doca;
    private double _Phi_0;
    private double _Radius;
    private double _Numwire;

    public Hit(int _Superlayer, int _Layer, int _Wire, double _Doca) {
        this._Superlayer = _Superlayer;
        this._Layer = _Layer;
        this._Wire = _Wire;
        this._Doca = _Doca;
        if(this._Superlayer == 0 && this._Layer == 0){calcRadiusAndPhi(32, 47);}
        if(this._Superlayer == 1 && this._Layer == 0){calcRadiusAndPhi(38, 56);}
        if(this._Superlayer == 1 && this._Layer == 1){calcRadiusAndPhi(42, 56);}
        if(this._Superlayer == 2 && this._Layer == 0){calcRadiusAndPhi(48, 72);}
        if(this._Superlayer == 2 && this._Layer == 1){calcRadiusAndPhi(52, 72);}
        if(this._Superlayer == 3 && this._Layer == 0){calcRadiusAndPhi(58, 87);}
        if(this._Superlayer == 3 && this._Layer == 1){calcRadiusAndPhi(62, 87);}
        if(this._Superlayer == 4 && this._Layer == 0){calcRadiusAndPhi(68, 99);}
    }

    private void calcRadiusAndPhi(double radius, double numwire){
        this._Phi_0 = (this._Wire-1)* Math.toRadians(360/numwire);
        this._Radius = radius;
        this._Numwire = numwire;
    }


    public String toString(){
        return "Hit : Superlayer : " + this._Superlayer + " Layer : " + this._Layer + " Wire : " + this._Wire;
    }

    @Override
    public int compareTo(Hit arg0) { //TODO: A refaire correctement
        if ((this._Superlayer == arg0._Superlayer && this._Layer == arg0._Layer && this._Wire > arg0._Wire)
            || (this._Superlayer == arg0._Superlayer && this._Layer > arg0._Layer)
            || (this._Superlayer > arg0._Superlayer)) {
            return 1;
        } else {
            return 0;
        }
    }

    public int get_Id() {
        return _Id;
    }

    public void set_Id(int _id) {
        _Id = _id;
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

    public double get_Phi_0() {
        return _Phi_0;
    }

    public void set_Phi_0(double _Phi_0) {
        this._Phi_0 = _Phi_0;
    }

    public double get_Radius() {
        return _Radius;
    }

    public void set_Radius(double _Radius) {
        this._Radius = _Radius;
    }

    public double get_Numwire() {
        return _Numwire;
    }

    public void set_Numwire(double _Numwire) {
        this._Numwire = _Numwire;
    }

}
