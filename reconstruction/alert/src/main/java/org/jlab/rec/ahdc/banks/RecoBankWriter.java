package org.jlab.rec.ahdc.banks;

import org.jlab.io.base.DataBank;
import org.jlab.io.base.DataEvent;
import org.jlab.rec.ahdc.hit.Hit;
import java.util.List;

public class RecoBankWriter {

    public DataBank fillAHDCHitsBank(DataEvent event, List<Hit> hitlist) {
        if (hitlist == null) {
            return null;
        }
        if (hitlist.size() == 0) {
            return null;
        }

        DataBank bank = event.createBank("AHDCRec::Hits", hitlist.size());

        for (int i = 0; i < hitlist.size(); i++) {

            bank.setShort("ID", i, (short) hitlist.get(i).get_Id());
            bank.setByte("layer", i, (byte) hitlist.get(i).get_Layer());
            bank.setByte("superlayer", i, (byte) hitlist.get(i).get_Superlayer());
            bank.setInt("wire", i, hitlist.get(i).get_Wire());
            bank.setDouble("Doca", i, hitlist.get(i).get_Doca());
        }

        return bank;

    }

}
