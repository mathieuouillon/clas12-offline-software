package org.jlab.rec.ahdc.PreCluster;

import org.jlab.rec.ahdc.Hit.Hit;

import java.util.ArrayList;
import java.util.List;

public class PreClusterFinder {

    private List<PreCluster> _AHDCPreCluster;

    public List<PreCluster> get_AHDCPreCluster() {
        return _AHDCPreCluster;
    }

    public void set_AHDCPreCluster(List<PreCluster> _AHDCPreCluster) {
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
    }
}
