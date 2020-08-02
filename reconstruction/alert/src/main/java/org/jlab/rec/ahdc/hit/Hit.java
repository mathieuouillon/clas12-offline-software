package org.jlab.rec.ahdc.hit;


public class Hit implements Comparable<Hit> {

    private int _Id;
    private int _Superlayer;
    private int _Layer;
    private int _Wire;
    private double _Doca;
    private boolean _Used = false;

    public Hit(int _Superlayer, int _Layer, int _Wire, double _Doca) {
        this._Superlayer = _Superlayer;
        this._Layer = _Layer;
        this._Wire = _Wire;
        this._Doca = _Doca;
    }

    public Boolean get_Used() {
        return _Used;
    }

    public void set_Used(Boolean _Used) {
        this._Used = _Used;
    }

    @Override
    public int compareTo(Hit arg0) {
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

}
