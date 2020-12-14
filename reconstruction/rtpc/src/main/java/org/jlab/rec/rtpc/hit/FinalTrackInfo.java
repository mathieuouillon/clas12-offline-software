/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.jlab.rec.rtpc.hit;

import Jama.Matrix;
import org.jlab.clas.tracking.trackrep.Helix;

/**
 *
 * @author davidpayette
 */
public class FinalTrackInfo {
    
    private double _px;
    private double _py;
    private double _pz;
    private double _vz;
    private double _tl;
    private double _dEdx;
    private double _theta;
    private double _phi;
    private int _numhits;
    private double _R;
    private double _A;
    private double _B;
    private double _chi2;
    private double _ADCsum;
    private double _d_rho;
    private double _phi_0;
    private double _kappa;
    private double _d_z;
    private double _tanL;
    private Matrix _cov;

    public Helix getKFHelix() {
        return KFHelix;
    }

    public void setKFHelix(Helix KFHelix) {
        this.KFHelix = KFHelix;
    }

    private Helix KFHelix;

    public double get_X0() {
        return _X0;
    }
    public void set_X0(double _X0) {
        this._X0 = _X0;
    }
    public double get_Y0() {
        return _Y0;
    }
    public void set_Y0(double _Y0) {
        this._Y0 = _Y0;
    }
    public double get_Z0() {
        return _Z0;
    }
    public void set_Z0(double _Z0) {
        this._Z0 = _Z0;
    }

    private double _X0;
    private double _Y0;
    private double _Z0;
    
    public FinalTrackInfo(){}
    
    public FinalTrackInfo(double px, double py, double pz, double vz, double theta, double phi, int numhits, double tl,
                          double ADCsum, double dEdx, double R, double A, double B, double chi2, double X0, double Y0, double Z0){
        _px = px;
        _py = py;
        _pz = pz;
        _vz = vz;
        _theta = theta;
        _phi = phi;
        _numhits = numhits;
        _tl = tl;
        _dEdx = dEdx;
        _R = R;
        _A = A;
        _B = B;
        _chi2 = chi2;
        _ADCsum = ADCsum;
        _X0 = X0;
        _Y0 = Y0;
        _Z0 = Z0;
    }
    
    public void set_px(double px){
        _px = px;
    }
    public void set_py(double py){
        _py = py;
    }
    public void set_pz(double pz){
        _pz = pz;
    }
    public void set_tl(double tl){
        _tl = tl;
    }
    public void set_theta(double theta){
        _theta = theta;
    }
    public void set_phi(double phi){
        _phi = phi;
    }
    public void set_numhits(int numhits){
        _numhits = numhits;
    }
    public void set_dEdx(double dEdx){
        _dEdx = dEdx;
    }
    public void set_vz(double vz){
        _vz = vz;
    }
    public void set_d_rho(double _d_rho) {
        this._d_rho = _d_rho;
    }
    public void set_phi_0(double _phi_0) {
        this._phi_0 = _phi_0;
    }
    public void set_kappa(double _kappa) {
        this._kappa = _kappa;
    }
    public void set_d_z(double _d_z) {
        this._d_z = _d_z;
    }
    public void set_tanL(double _tanL) {
        this._tanL = _tanL;
    }
    public void set_cov(Matrix _cov) {
        this._cov = _cov;
    }

    public double get_px(){
        return _px;
    }   
    public double get_py(){
        return _py;
    }
    public double get_pz(){
        return _pz;
    }
    public double get_tl(){
        return _tl;
    }
    public double get_theta(){
        return _theta;
    }
    public double get_phi(){
        return _phi;
    }
    public int get_numhits(){
        return _numhits;
    }
    public double get_dEdx(){
        return _dEdx;
    }
    public double get_vz(){
        return _vz;
    }
    public double get_R(){
        return _R;
    }
    public double get_A(){
        return _A;
    }
    public double get_B(){
        return _B;
    }
    public double get_chi2(){
        return _chi2;
    }
    public double get_ADCsum(){
        return _ADCsum;
    }
    public double get_d_rho() {
        return _d_rho;
    }
    public double get_phi_0() {
        return _phi_0;
    }
    public double get_kappa() {
        return _kappa;
    }
    public double get_d_z() {
        return _d_z;
    }
    public double get_tanL() {
        return _tanL;
    }
    public Matrix get_cov() {
        return _cov;
    }

}
