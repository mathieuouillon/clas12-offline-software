package org.jlab.rec.ahdc.track;

import org.jlab.rec.ahdc.cross.Cross;
import org.jlab.rec.ahdc.fit.HelicalTrackFitter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TrackSeeder {

    public TrackSeeder() {

    }

    private List<ArrayList<Cross>> seedlist = new ArrayList<ArrayList<Cross>>();

    public void FindSeedCrossList(List<Cross> crosses) {
        List<Cross> CrossOnFirstLayer = new ArrayList<Cross>();
        for(Cross cross : crosses){
            if (cross.get_Radius()<38){
                CrossOnFirstLayer.add(cross);
            }
        }

        for(Cross cross : CrossOnFirstLayer){
            ArrayList<Cross> TempSeed = new ArrayList<Cross>();
            TempSeed.add(cross);
            for (Cross cross1 : crosses)
            if (cross1.get_Radius()>42 && cross1.get_Radius()<48 && cross1.get_Phi() > (TempSeed.get(TempSeed.size()-1).get_Phi() - 5)
                    && (cross1.get_Phi() < TempSeed.get(TempSeed.size()-1).get_Phi() +5)){
                TempSeed.add(cross1);
            }
            seedlist.add(TempSeed);
            TempSeed.clear();
        }
        List<Track> cands = new ArrayList<Track>();
        for (ArrayList<Cross> SeedCross : seedlist) {
            Track cand = fitSeed(SeedCross);
            cands.add(cand);
        }
    }

    public Track fitSeed(ArrayList<Cross> SeedCross){
        Track cand = null;
        HelicalTrackFitter fitTrk = new HelicalTrackFitter();
        List<Double> X = new ArrayList<Double>();
        List<Double> Y = new ArrayList<Double>();
        List<Double> Z = new ArrayList<Double>();
        List<Double> Rho = new ArrayList<Double>();
        List<Double> ErrZ = new ArrayList<Double>();
        List<Double> ErrRho = new ArrayList<Double>();
        List<Double> ErrRt = new ArrayList<Double>();
        for(Cross cross : SeedCross){
            X.add(cross.get_Point().x());
            Y.add(cross.get_Point().y());
            Z.add(cross.get_Point().z());
            Rho.add(Math.sqrt(cross.get_Point().x()*cross.get_Point().x()+cross.get_Point().y()*cross.get_Point().y()));
            ErrZ.add(cross.get_PointErr().z());
            ErrRho.add(Math.sqrt(cross.get_PointErr().x()*cross.get_PointErr().x()+cross.get_PointErr().y()*cross.get_PointErr().y()));
            ErrRt.add(Math.sqrt(cross.get_PointErr().x()*cross.get_PointErr().x()+cross.get_PointErr().y()*cross.get_PointErr().y()));
        }
        fitTrk.fit(X, Y, Z, Rho, ErrRt, ErrRho, ErrZ);
        if (fitTrk.get_helix() == null) { return null; }
        cand = new Track(fitTrk.get_helix());
        return cand;
    }
}
