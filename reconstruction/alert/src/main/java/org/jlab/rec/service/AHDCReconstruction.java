package org.jlab.rec.service;

import org.jlab.clas.reco.ReconstructionEngine;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.ahdc.Distance.Distance;
import org.jlab.rec.ahdc.Hit.HitReader;
import org.jlab.rec.ahdc.Cluster.Cluster;
import org.jlab.rec.ahdc.Cluster.ClusterFinder;
import org.jlab.rec.ahdc.Hit.Hit;
import org.jlab.rec.ahdc.HoughTransform.HoughTransform;
import org.jlab.rec.ahdc.PreCluster.PreCluster;
import org.jlab.rec.ahdc.PreCluster.PreClusterFinder;
import org.jlab.rec.ahdc.Track.Track;

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

        String method = "hough";

        if(event.hasBank("AHDC::adc")) {

            // I) Read hit
            HitReader hitRead = new HitReader();
            hitRead.fetch_AHDCHits(event);
            List<Hit> AHDC_Hits = hitRead.get_AHDCHits();

            // II) Create PreCluster
            PreClusterFinder preclusterfinder = new PreClusterFinder();
            preclusterfinder.findPreCluster(AHDC_Hits);
            List<PreCluster> AHDC_PreClusters = preclusterfinder.get_AHDCPreClusters();

            // III) Create Cluster
            ClusterFinder clusterfinder = new ClusterFinder();
            clusterfinder.findCluster(AHDC_PreClusters);
            List<Cluster> AHDC_Clusters = clusterfinder.get_AHDCClusters();

            System.out.println(AHDC_Clusters);

            // IV) Track Finder
            List<Track> AHDC_Tracks = new ArrayList<>();
            if(method.equals("distance")){
                // IV) a) Distance method
                Distance distance = new Distance();
                distance.find_track(AHDC_Clusters);
                AHDC_Tracks = distance.get_AHDCTracks();
            }

            else if(method.equals("hough")){
                // IV) b) Hough Transform method
                HoughTransform houghtransform = new HoughTransform();
                houghtransform.find_tracks(AHDC_Clusters);
                AHDC_Tracks = houghtransform.get_AHDCTracks();
            }

            System.out.println(AHDC_Tracks);
        }

        System.out.println();

        return true;
    }

    public static void main(String[] args){

    }
}


