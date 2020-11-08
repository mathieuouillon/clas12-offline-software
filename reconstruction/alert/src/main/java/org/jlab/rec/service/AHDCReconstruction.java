package org.jlab.rec.service;

import org.jlab.clas.reco.ReconstructionEngine;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.ahdc.Hit.HitReader;
import org.jlab.rec.ahdc.Cluster.Cluster;
import org.jlab.rec.ahdc.Cluster.ClusterFinder;
import org.jlab.rec.ahdc.Cross.Cross;
import org.jlab.rec.ahdc.Cross.CrossMaker;
import org.jlab.rec.ahdc.Hit.Hit;
import org.jlab.rec.ahdc.PreCluster.PreCluster;
import org.jlab.rec.ahdc.PreCluster.PreClusterFinder;

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
        // I) Read hit
        HitReader hitRead = new HitReader();
        hitRead.fetch_AHDCHits(event);
        List<Hit> ahdc_hits = hitRead.get_AHDCHits();

        // II) Create PreCluster
        PreClusterFinder preclusterfinder = new PreClusterFinder();
        preclusterfinder.findPreCluster(ahdc_hits);
        ArrayList<PreCluster> ahdc_precluster = preclusterfinder.get_AHDCPreCluster();

        ClusterFinder clusterfinder = new ClusterFinder();
        clusterfinder.findCluster(ahdc_precluster);
        ArrayList<Cluster> ahdc_cluster = clusterfinder.get_AHDCClusters();

        CrossMaker crossmaker = new CrossMaker();
        crossmaker.findCross(ahdc_cluster);
        ArrayList<Cross> ahdc_cross = crossmaker.get_Crosses();

        return true;
    }

    public static void main(String[] args){

    }
}


