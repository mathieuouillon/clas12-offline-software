package org.jlab.rec.ahdc.Hit;

import org.jlab.io.base.DataBank;
import org.jlab.io.base.DataEvent;

import java.util.ArrayList;
import java.util.List;

public class HitReader {

    private List<Hit> _AHDCHits;

    public HitReader() {
    }

    public List<Hit> get_AHDCHits() {
        return _AHDCHits;
    }

    public void set_AHDCHits(List<Hit> _AHDCHits) {
        this._AHDCHits = _AHDCHits;
    }

    public void fetch_AHDCHits(DataEvent event){

        List<Hit> hits = new ArrayList<Hit>();

        DataBank bankDGTZ = event.getBank("AHDC::adc");
        int rows = bankDGTZ.rows();;
        int[] id = new int[rows];
        int[] superlayer = new int[rows];
        int[] layer = new int[rows];
        int[] wire = new int[rows];
        float[] doca = new float[rows];

        if (event.hasBank("AHDC::adc")) {
            for (int i = 0; i < rows; i++) {
                id[i] = i + 1;
                superlayer[i] = bankDGTZ.getByte("superlayer", i);
                layer[i] = bankDGTZ.getByte("layer", i);
                wire[i] = bankDGTZ.getShort("wire", i);
                doca[i] = bankDGTZ.getFloat("doca", i);

                // create the hit object
                Hit hit = new Hit(superlayer[i], layer[i], wire[i], doca[i]);
                hit.set_Id(id[i]);
                // add this hit
                hits.add(hit);
            }
        }
        this.set_AHDCHits(hits);
    }

    public List<TrueHit> fetch_TrueAHDCHits(DataEvent event){
        List<TrueHit> truehits = new ArrayList<TrueHit>();
        DataBank bankSIMU = event.getBank("MC::True");
        int rows = bankSIMU.rows();
        int[] pid = new int[rows];
        float[] x_true = new float[rows];
        float[] y_true = new float[rows];
        float[] z_true = new float[rows];
        float[] trackE = new float[rows];

        if(event.hasBank("MC::True")){
            for (int i = 0; i < rows; i++) {
                pid[i] = bankSIMU.getInt("pid",i);
                x_true[i] = bankSIMU.getFloat("avgX",i);
                y_true[i] = bankSIMU.getFloat("avgY",i);
                z_true[i] = bankSIMU.getFloat("avgZ",i);
                trackE[i] = bankSIMU.getFloat("trackE", i);
                TrueHit truehit = new TrueHit(pid[i], x_true[i], y_true[i], z_true[i], trackE[i]);
                truehits.add(truehit);
            }
        }
        return truehits;
    }
}