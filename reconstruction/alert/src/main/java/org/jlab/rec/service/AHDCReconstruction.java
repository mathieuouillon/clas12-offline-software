package org.jlab.rec.service;

import Jama.Matrix;
import org.jlab.clas.reco.ReconstructionEngine;
import org.jlab.clas.swimtools.Swim;
import org.jlab.clas.tracking.kalmanfilter.helical.KFitter;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.ahdc.Distance.Distance;
import org.jlab.rec.ahdc.HelixFit.HelixFitJava;
import org.jlab.rec.ahdc.HelixFit.HelixFitObject;
import org.jlab.rec.ahdc.Hit.HitReader;
import org.jlab.rec.ahdc.Cluster.Cluster;
import org.jlab.rec.ahdc.Cluster.ClusterFinder;
import org.jlab.rec.ahdc.Hit.Hit;
import org.jlab.rec.ahdc.Hit.TrueHit;
import org.jlab.rec.ahdc.HoughTransform.HoughTransform;
import org.jlab.rec.ahdc.KalmanFilter.KalmanFilter;
import org.jlab.rec.ahdc.PreCluster.PreCluster;
import org.jlab.rec.ahdc.PreCluster.PreClusterFinder;
import org.jlab.rec.ahdc.RecUtilities;
import org.jlab.rec.ahdc.Track.Track;
import org.jlab.clas.tracking.trackrep.Helix;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AHDCReconstruction extends ReconstructionEngine {

    public AHDCReconstruction() {
        super("ALERT","ouillon","0.0.1");
    }

    @Override
    public boolean init() {
        return true;
    }

    /**
     * Command to run the engine :
     *          ./coatjava/bin/recon-util org.jlab.rec.service.AHDCReconstruction org.jlab.clas.swimtools.MagFieldsEngine -i input.hipo -o output.hipo
     * @param event
     * @return
     */
    @Override
    public boolean processDataEvent(DataEvent event) {

        String method = "distance";

/*        if (event.hasBank("AHDC::adc")) {

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

            // IV) Track Finder
            List<Track> AHDC_Tracks = new ArrayList<>();
            if (method.equals("distance")) {
                // IV) a) Distance method
                Distance distance = new Distance();
                distance.find_track(AHDC_Clusters);
                AHDC_Tracks = distance.get_AHDCTracks();
            } else if (method.equals("hough")) {
                // IV) b) Hough Transform method
                HoughTransform houghtransform = new HoughTransform();
                houghtransform.find_tracks(AHDC_Clusters);
                AHDC_Tracks = houghtransform.get_AHDCTracks();
            }

        }*/

        if (event.hasBank("MC::True")) {

            //Kalman filter and Global Fit but with Geant4 data
            Swim swimmer = new Swim();
            RecUtilities recUtil = new RecUtilities();
            HitReader hitRead = new HitReader();

            // Lecture des infos geant4 des hits
            List<TrueHit> trueHitList = hitRead.fetch_TrueAHDCHits(event);

            // Nombre de points
            int nbofpoint = 0;
            for (TrueHit trueHit : trueHitList) {
                if (trueHit.get_Pid() == 2212) {
                    nbofpoint++;
                }
            }

            // szPos[nbofpoint][3] x y z pour chaque point
            double[][] szPos = new double[nbofpoint][3];
            int j = 0;
            for (TrueHit trueHit : trueHitList) {
                if (trueHit.get_Pid() == 2212) {
                    szPos[j][0] = trueHit.get_X();
                    szPos[j][1] = trueHit.get_Y();
                    szPos[j][2] = trueHit.get_Z();
                    j++;
                }
            }

            // Global fit
            HelixFitJava h = new HelixFitJava();
            HelixFitObject ho = h.HelixFit(nbofpoint, szPos, 0);

            // Kalman Filter
            // Creation d'une helice pour le filtre de Kalman
            Helix hlx = new Helix(ho.get_X0(), ho.get_Y0(), ho.get_Z0(), ho.get_px(), ho.get_py(), ho.get_pz(), 1, 5, 0, 0, Helix.Units.MM);

            double omega = hlx.getOmega();
            double tanL = hlx.getTanL();
            double phi0 = hlx.getPhi0();
            double d0 = hlx.getD0();
            double z0 = hlx.getZ0();

            Matrix cov = new Matrix(5, 5);
            cov.set(0, 0, 1);
            cov.set(1, 1, 1);
            cov.set(2, 2, 1);
            cov.set(3, 3, 1);
            cov.set(4, 4, 1);

//            cov.set(0, 0, d0 * d0);
//            cov.set(1, 0, phi0 * d0);
//            cov.set(0, 1, phi0 * d0);
//            cov.set(2, 0, omega * d0);
//            cov.set(0, 2, omega * d0);
//            cov.set(1, 1, phi0 * phi0);
//            cov.set(1, 2, phi0 * omega);
//            cov.set(2, 1, phi0 * omega);
//            cov.set(2, 2, omega * omega);
//            cov.set(3, 3, z0 * z0);
//            cov.set(4, 4, tanL * tanL);

            KFitter kf = new KFitter(hlx, cov, event, swimmer, 0, 0, 0, recUtil.setMeasVecs(trueHitList));
            kf.runFitter(swimmer);

            double mom_gf = ho.get_Mom();
            double pt = Math.abs(1. / kf.finalStateVec.kappa);
            double pz = pt * kf.finalStateVec.tanL;
            double mom_kf = Math.sqrt(pt * pt + pz * pz);

            System.out.println("mom_kf = " + mom_kf);
            System.out.println("finalStateVec : kappa = " + kf.finalStateVec.kappa + " tanL = " + kf.finalStateVec.tanL);

            try {
                BufferedWriter writer = new BufferedWriter(new FileWriter("filename.txt", true));
                writer.write("mom_gf = " + mom_gf + '\n');
                writer.write("mom_kf = " + mom_kf + '\n');
                writer.write('\n');
                writer.close();
            } catch (IOException e) {
                System.out.println("An error occurred.");
                e.printStackTrace();
            }

            /*for(Track track : AHDC_Tracks){
                //Create szPos[any number>3][3] xyz array for HelixFit
                int nb_of_point = track.get_Clusters().size();
                double[][] szPos = new double[nb_of_point][3];
                int i = 0;
                for(Cluster cluster : track.get_Clusters()){
                    szPos[i][0] = cluster.get_X();
                    szPos[i][1] = cluster.get_Y();
                    szPos[i][2] = cluster.get_Z();
                    i++;
                }
                }*/

        }
        return true;
    }

    public static void main(String[] args){

    }
}


