package org.jlab.rec.service;

import org.jlab.clas.reco.ReconstructionEngine;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.ahdc.Hit.HitReader;
import org.jlab.rec.ahdc.Cluster.Cluster;
import org.jlab.rec.ahdc.Cluster.ClusterFinder;
import org.jlab.rec.ahdc.Cross.Cross;
import org.jlab.rec.ahdc.Cross.CrossMaker;
import org.jlab.rec.ahdc.Hit.Hit;

import java.util.ArrayList;
import java.util.List;

public class AHDCReconstruction extends ReconstructionEngine {

    public AHDCReconstruction() {
        super("ALERT","ouillon","0.0.1");
    }

    @Override
    public boolean init() {
        return true;
    }

    @Override
    public boolean processDataEvent(DataEvent event) {

        HitReader hitRead = new HitReader();
        hitRead.fetch_AHDCHits(event);
        List<Hit> ahdc_hits = hitRead.get_AHDCHits();

        ClusterFinder clusterfinder = new ClusterFinder();
        clusterfinder.findClusters(ahdc_hits);
        ArrayList<Cluster> clusters = clusterfinder.get_Clusters();

        CrossMaker crossmaker = new CrossMaker();
        crossmaker.findCross(clusters);
        ArrayList<Cross> crosses = crossmaker.getCrosses();

        return true;
    }

    public static void main(String[] args){

    }
}


