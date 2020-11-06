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
        System.out.println("-------------------------------------------------------------");
        System.out.println("++++++++++++++++++++++++++New event++++++++++++++++++++++++++");
        System.out.println("-------------------------------------------------------------");
        RecoBankWriter rbc = new RecoBankWriter();

        HitReader hitRead = new HitReader();
        hitRead.fetch_AHDCHits(event);
        List<Hit> ahdc_hits = hitRead.get_AHDCHits();

        ClusterFinder clusterfinder = new ClusterFinder();
        clusterfinder.findClusters(ahdc_hits);
        ArrayList<Cluster> clusters = clusterfinder.get_Clusters();

        CrossMaker crossmaker = new CrossMaker();
        crossmaker.findCross(clusters);
        ArrayList<Cross> crosses = crossmaker.getCrosses();
//        for(Cross cross : crosses){
//            System.out.println("X : "+ cross.get_Point().x());
//            System.out.println("Y : "+ cross.get_Point().y());
//            System.out.println("Z : "+ cross.get_Point().z());
//            System.out.println(" ");
//        }

        return true;
    }

    public static void main(String[] args){

    }
}


