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

        if (!event.hasBank("AHDC::adc")) {
            System.err.println("there is no AHDC bank ");
            _AHDCHits = new ArrayList<Hit>();
            return;
        }

        List<Hit> hits = new ArrayList<Hit>();

        DataBank bankDGTZ = event.getBank("AHDC::adc");
        int rows = bankDGTZ.rows();;
        int[] id = new int[rows];
        int[] superlayer = new int[rows];
        int[] layer = new int[rows];
        int[] wire = new int[rows];
        float[] Doca = new float[rows];

        if (event.hasBank("AHDC::adc")) {
            for (int i = 0; i < rows; i++) {

                if (bankDGTZ.getFloat("adc", i) < 0) {
                    continue;
                }

                id[i] = i + 1;
                superlayer[i] = bankDGTZ.getByte("superlayer", i);
                layer[i] = bankDGTZ.getByte("layer", i);
                wire[i] = bankDGTZ.getShort("wire", i);
                Doca[i] = bankDGTZ.getFloat("time", i);

                // create the hit object
                Hit hit = new Hit(superlayer[i], layer[i], wire[i], Doca[i]);
                hit.set_Id(id[i]);
                // add this hit
                hits.add(hit);
            }
        }
        this.set_AHDCHits(hits);
    }
}
