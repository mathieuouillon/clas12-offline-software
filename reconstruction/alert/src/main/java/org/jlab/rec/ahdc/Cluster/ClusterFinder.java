package org.jlab.rec.ahdc.Cluster;

import org.jlab.rec.ahdc.Hit.Hit;
import org.jlab.rec.ahdc.PreCluster.PreCluster;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Cluster are form by 2 hit in layer with different stereo angle to compute z coordinates
 */
public class ClusterFinder {

    public ClusterFinder() {}

    public ArrayList<Cluster> get_AHDCClusters() {
        return _AHDCClusters;
    }

    public void set_AHDCClusters(ArrayList<Cluster> _AHDCClusters) {
        this._AHDCClusters = _AHDCClusters;
    }

    ArrayList<Cluster> _AHDCClusters = new ArrayList<Cluster>();

    public void findCluster(ArrayList<PreCluster> ahdc_precluster){
        for(PreCluster precluster : ahdc_precluster){
            if(precluster.get_Superlayer() == 0 && precluster.get_Layer() == 0){
                for(PreCluster otherprecluster : ahdc_precluster) {
                    if(otherprecluster.get_Superlayer() == 1 && otherprecluster.get_Layer() == 0) {
                        if(precluster.get_Phi_0() < otherprecluster.get_Phi_0() && precluster.get_Phi_0() + Math.toRadians(20) > otherprecluster.get_Phi_0() - Math.toRadians(20) ){
                            _AHDCClusters.add(new Cluster(precluster,otherprecluster));
                        }
                    else if(precluster.get_Phi_0() + Math.toRadians(20) > 2*Math.PI){
                            if(otherprecluster.get_Phi_0() < precluster.get_Phi_0() && otherprecluster.get_Phi_0() - Math.toRadians(20) < 0){
                                _AHDCClusters.add(new Cluster(precluster,otherprecluster));
                            }
                        }
                    }
                }
            }


            if(precluster.get_Superlayer() == 1 && precluster.get_Layer() == 1){
                for(PreCluster otherprecluster : ahdc_precluster) {
                    if(otherprecluster.get_Superlayer() == 2 && otherprecluster.get_Layer() == 0) {
                        if(precluster.get_Phi_0() > otherprecluster.get_Phi_0() && precluster.get_Phi_0() - Math.toRadians(20) < otherprecluster.get_Phi_0() + Math.toRadians(20) ){
                            _AHDCClusters.add(new Cluster(precluster,otherprecluster));
                        }
                        else if(precluster.get_Phi_0() - Math.toRadians(20) < 0 ){
                            if(otherprecluster.get_Phi_0() > precluster.get_Phi_0() && otherprecluster.get_Phi_0() + Math.toRadians(20) > 2*Math.PI){
                                _AHDCClusters.add(new Cluster(precluster,otherprecluster));
                            }
                        }
                    }
                }
            }


            if(precluster.get_Superlayer() == 2 && precluster.get_Layer() == 1){
                for(PreCluster otherprecluster : ahdc_precluster) {
                    if(otherprecluster.get_Superlayer() == 3 && otherprecluster.get_Layer() == 0) {
                        if(precluster.get_Phi_0() < otherprecluster.get_Phi_0() && precluster.get_Phi_0() + Math.toRadians(20) > otherprecluster.get_Phi_0() - Math.toRadians(20) ){
                            _AHDCClusters.add(new Cluster(precluster,otherprecluster));
                        }
                        else if(precluster.get_Phi_0() + Math.toRadians(20) > 2*Math.PI){
                            if(otherprecluster.get_Phi_0() < precluster.get_Phi_0() && otherprecluster.get_Phi_0() - Math.toRadians(20) < 0){
                                _AHDCClusters.add(new Cluster(precluster,otherprecluster));
                            }
                        }
                    }
                }
            }



            if(precluster.get_Superlayer() == 3 && precluster.get_Layer() == 1){
                for(PreCluster otherprecluster : ahdc_precluster) {
                    if(otherprecluster.get_Superlayer() == 4 && otherprecluster.get_Layer() == 0) {
                        if(precluster.get_Phi_0() > otherprecluster.get_Phi_0() && precluster.get_Phi_0() - Math.toRadians(20) < otherprecluster.get_Phi_0() + Math.toRadians(20) ){
                            _AHDCClusters.add(new Cluster(precluster,otherprecluster));
                        }
                        else if(precluster.get_Phi_0() - Math.toRadians(20) < 0 ){
                            if(otherprecluster.get_Phi_0() > precluster.get_Phi_0() && otherprecluster.get_Phi_0() + Math.toRadians(20) > 2*Math.PI){
                                _AHDCClusters.add(new Cluster(precluster,otherprecluster));
                            }
                        }
                    }
                }
            }
        }
    }
}



