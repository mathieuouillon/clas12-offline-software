/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.jlab.rec.rtpc.hit;

/**
 *
 * @author davidpayette
 */
public class HelixFitObject {
    private double _Rho;
    private double _A;
    private double _B;
    private double _Phi;
    private double _Theta;
    private double _X0;
    private double _Y0;
    private double _Z0;
    private double _DCA;
    private double _Chi2;
    private double _magfield;

    public double get_Omega() {
        return _Omega;
    }

    public void set_Omega(double _Omega) {
        this._Omega = _Omega;
    }

    public double get_TanL() {
        return _TanL;
    }

    public void set_TanL(double _TanL) {
        this._TanL = _TanL;
    }

    public double get_Phi0() {
        return _Phi0;
    }

    public void set_Phi0(double _Phi0) {
        this._Phi0 = _Phi0;
    }

    public double get_D0() {
        return _D0;
    }

    public void set_D0(double _D0) {
        this._D0 = _D0;
    }

    private double _Omega;
    private double _TanL;
    private double _Phi0;
    private double _D0;

    
    public HelixFitObject(){
        //default constructor
    }
    
    public HelixFitObject(double Rho, double A, double B, double Phi, double Theta, double X0, double Y0, double Z0, double DCA, double Chi2,
                          double omega, double tanL, double phi0, double d0){
        _Rho = Rho;
        _A = A;
        _B = B;
        _Phi = Phi;
        _Theta = Theta;
        _X0 = X0;
        _Y0 = Y0;
        _Z0 = Z0;
        _DCA = DCA;
        _Chi2 = Chi2;
        _magfield = 50;
        _Omega = omega;
        _TanL = tanL;
        _Phi0 = phi0;
        _D0 = d0;

    }
    public double get_Rho(){
        return _Rho;
    }
    public void set_Rho(double Rho){
        _Rho = Rho;
    }
    public double get_A(){
        return _A;
    }
    public void set_A(double A){
        _A = A;
    }
    public double get_B(){
        return _B;
    }    
    public void set_B(double B){
        _B = B;
    }
    public double get_Phi(){
        return _Phi;
    }       
    public void set_Phi(double Phi){
        _Phi = Phi;
    }
    public double get_Theta(){
        return _Theta;
    }   
    public void set_Theta(double Theta){
        _Theta = Theta;
    }
    public double get_X0(){
        return _X0;
    }    
    public double get_Y0(){
        return _Y0;
    }    
    public double get_Z0(){
        return _Z0;
    }   
    public void set_Z0(double Z0){
        _Z0 = Z0;
    }
    public double get_DCA(){
        return _DCA;
    }    
    public double get_Chi2(){
        return _Chi2;
    }
    public double get_Mom(){
        return 0.3*_magfield*Math.abs(_Rho)/(10*Math.sin(Math.toRadians(_Theta)));
    }
    public double get_px(){
        return get_Mom()*Math.cos(Math.toRadians(_Phi))*Math.sin(Math.toRadians(_Theta));
    }
    public double get_py(){
        return get_Mom()*Math.sin(Math.toRadians(_Phi))*Math.sin(Math.toRadians(_Theta));
    }
    public double get_pz(){
        return get_Mom()*Math.cos(Math.toRadians(_Theta));
    }
    public double get_trackl(){
        return 0;
    }
    public double get_dEdx(){
        return 0;
    }
    public void set_magfield(double magfield){
        _magfield = magfield;
    }
    
}