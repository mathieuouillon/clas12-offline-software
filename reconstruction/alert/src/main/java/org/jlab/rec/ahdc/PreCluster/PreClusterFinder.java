package org.jlab.rec.ahdc.PreCluster;

import org.jlab.rec.ahdc.Hit.Hit;

import java.util.ArrayList;
import java.util.List;

public class PreClusterFinder {

    private ArrayList<PreCluster> _AHDCPreCluster;
    public ArrayList<PreCluster> get_AHDCPreCluster() {
        return _AHDCPreCluster;
    }

    public void set_AHDCPreCluster(ArrayList<PreCluster> _AHDCPreCluster) {
        this._AHDCPreCluster = _AHDCPreCluster;
    }

    public PreClusterFinder(){

    }

    public void findPreCluster(List<Hit> ahdc_hits) {
        ArrayList<Hit> run = new ArrayList<Hit>();
        List<ArrayList<Hit>> result = new ArrayList<ArrayList<Hit>>();
        Hit expect = ahdc_hits.get(0);

        for(Hit hit : ahdc_hits){
            if (hit == expect) {
                run.add(hit);
            }
            else {
                run = new ArrayList<Hit>();
                run.add(hit);
                result.add(run);
            }
            expect = hit;

        }

        for(ArrayList<Hit> list : result ){
            double pre_phi = 0;
            int count = 0;
            for(Hit hit : list){
                pre_phi += hit.get_Phi_0() ;
                count ++;
            }
            pre_phi /= count;
            _AHDCPreCluster.add(new PreCluster( list.get(0).get_Superlayer(), list.get(0).get_Layer(), 0, list.get(0).get_Radius(), pre_phi ));
        }
    }
}
