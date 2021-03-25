package org.jlab.rec.service;

import org.jlab.clas.reco.ReconstructionEngine;
import org.jlab.clas.swimtools.Swim;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.ahdc.HelixFit.HelixFitJava;
import org.jlab.rec.ahdc.HelixFit.HelixFitObject;
import org.jlab.rec.ahdc.Hit.HitReader;
import org.jlab.rec.ahdc.Hit.TrueHit;
import org.jlab.rec.ahdc.KalmanFilter.KalmanFilter;
import org.jlab.rec.ahdc.RecUtilities;

import java.util.List;

public class AHDCReconstruction extends ReconstructionEngine {

    public AHDCReconstruction() {
        super("ALERT", "ouillon", "0.0.1");
    }

    @Override
    public boolean init() {
        return true;
    }

    /**
     * Command to run the engine :
     * ./coatjava/bin/recon-util org.jlab.rec.service.AHDCReconstruction org.jlab.clas.swimtools.MagFieldsEngine -i input.hipo -o output.hipo
     * > - run
     * > mvn install
     * > in the reconstruction code directory. In your case, it would be clas12-offline-software/reconstruction/alert.
     * This will recreate the library jar in the target folder in the same directory.
     * > - If you want to use it with recon-util, you would then need to copy it to the coatjava folder as follows
     * (from the alert directory)
     * > cp target/clas12detector-alert-1.0-SNAPSHOT.jar ../../coatjava/lib/services/.
     * > - then you can run recon-util specifying the name of the service.
     * @param event Hipo DataEvent
     * @return 0
     */
    @Override
    public boolean processDataEvent(DataEvent event) {

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
            HelixFitObject ho = h.HelixFit(nbofpoint, szPos, 1);

            trueHitList.add(0, new TrueHit(2212, ho.get_X0(), ho.get_Y0(), ho.get_Z0(), 0));

            // Kalman Filter
            double[] starting = {ho.get_X0(), ho.get_Y0(), ho.get_Z0(), ho.get_px() / 1000, ho.get_py() / 1000, ho.get_pz() / 1000};
            if (ho.get_Mom() > 70. && ho.get_Mom() < 120. && Math.abs(starting[5]) < 0.07) {
                KalmanFilter kf = new KalmanFilter(swimmer);
                kf.runKalmanFilter(starting, recUtil.setMeasVecsGeant4(trueHitList), ho.get_Mom(), ho.get_Phi(), ho.get_Theta());
            }
        }

        return true;
    }
}


