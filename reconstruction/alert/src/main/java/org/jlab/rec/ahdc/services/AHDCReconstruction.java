package org.jlab.rec.ahdc.services;

import org.jlab.clas.reco.ReconstructionEngine;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.ahdc.banks.RecoBankWriter;
import org.jlab.rec.ahdc.banks.HitReader;
import org.jlab.rec.ahdc.cluster.Cluster;
import org.jlab.rec.ahdc.cluster.ClusterFinder;
import org.jlab.rec.ahdc.cross.Cross;
import org.jlab.rec.ahdc.cross.CrossMaker;
import org.jlab.rec.ahdc.hit.Hit;

import java.util.ArrayList;
import java.util.List;

public class AHDCReconstruction extends ReconstructionEngine {

    public AHDCReconstruction() {
        super("ALERT","mpaolone","1.0");
    }

    @Override
    public boolean init() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public boolean processDataEvent(DataEvent event) {
        RecoBankWriter rbc = new RecoBankWriter();

        HitReader hitRead = new HitReader();
        hitRead.fetch_AHDCHits(event);

        List<Hit> hits = new ArrayList<Hit>();

        List<Hit> ahdc_hits = hitRead.get_AHDCHits();
        for (int i = 0; i < ahdc_hits.size(); i++) {
            System.out.println("Wire Number : " + ahdc_hits.get(i).get_Wire());
        }
        int MAXAHDCHITS = 100000;
        if(ahdc_hits.size()> MAXAHDCHITS)
            return true;
        if (ahdc_hits != null && ahdc_hits.size() > 0) {
            hits.addAll(ahdc_hits);
        }

        ClusterFinder clusterfinder = new ClusterFinder();
        clusterfinder.findClusters(ahdc_hits);
        ArrayList<Cluster> clusters = clusterfinder.getClusters();


        CrossMaker crossmaker = new CrossMaker();
        crossmaker.findCross(clusters);
        ArrayList<Cross> crosses = crossmaker.getCrosses();
        return true;
    }

    public static void main(String[] args){

    }
}


