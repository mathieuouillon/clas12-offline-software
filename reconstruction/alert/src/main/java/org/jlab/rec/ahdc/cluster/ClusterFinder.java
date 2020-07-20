package org.jlab.rec.ahdc.cluster;

import org.jlab.rec.ahdc.hit.Hit;
import java.util.ArrayList;
import java.util.List;

/**
 * Cluster are form by 2 hit in layer with different stereo angle to compute z coordinates
 */
public class ClusterFinder {


    public ClusterFinder() {
    }

    ArrayList<Cluster> clusters = new ArrayList<Cluster>();

    public void findClusters(List<Hit> allhits){

        for(Hit hit : allhits){
            if(hit.get_Superlayer()*10 + hit.get_Layer() == 00) {      //hit on SL#0 and L#0 combine with hit on SL#1 L#0
                CreateACluster(hit,allhits,47,56,32,38);
            }
            if(hit.get_Superlayer()*10 + hit.get_Layer() == 11) {
                CreateACluster(hit,allhits,56,72,42,48);
            }
            if(hit.get_Superlayer()*10 + hit.get_Layer() == 21) {
                CreateACluster(hit,allhits,72,87,52,58);
            }
            if(hit.get_Superlayer()*10 + hit.get_Layer() == 31) {
                CreateACluster(hit,allhits,87,99,62,68);
            }
        }
    }

    private void CreateACluster(Hit hit,List<Hit> allhits, int num_wire, int num_wire_combinehit, int radius, int radiuscombinehit){

        ArrayList<Hit> cluster = new ArrayList<Hit>();
        cluster.add(hit);
        for(Hit hit1 : allhits){
            if(hit1.get_Superlayer() == hit.get_Superlayer() && hit1.get_Layer() == hit.get_Layer() && hit1.get_Wire() == hit.get_Wire()+1){
                cluster.add(hit1);
            }
            if(hit1.get_Superlayer() == hit.get_Superlayer() && hit1.get_Layer() == hit.get_Layer() && hit1.get_Wire() == hit.get_Wire()-1 ){
                cluster.add(hit1);
            }
            double distanceBetweenTwoWires = 2 * Math.PI / num_wire;
            double longueurdarc = hit.get_Wire() * distanceBetweenTwoWires;
            double AngleForThisWire = longueurdarc / radius;
            double longeurdarcforcombinehit = AngleForThisWire * radiuscombinehit;
            double distanceBetweenTwowirescombinehit = 2 * Math.PI / num_wire_combinehit;
            int combinewire = (int) Math.floor(longeurdarcforcombinehit / distanceBetweenTwowirescombinehit);
            for (int i = 1; i < 6; i++) {
                if (hit1.get_Superlayer() == hit.get_Superlayer() && hit1.get_Layer() == hit.get_Layer() && hit1.get_Wire() == combinewire + i) {
                    cluster.add(hit1);
                }
                if (hit1.get_Superlayer() == hit.get_Superlayer() && hit1.get_Layer() == hit.get_Layer() && hit1.get_Wire() == combinewire - i) {
                    cluster.add(hit1);
                }
            }
        }
        for (Hit hit1 : cluster){
            if(hit1.get_Superlayer()!=hit.get_Superlayer()){
                double a = Math.tan(-0.174533)/(2*radiuscombinehit); // combine hit
                double b = Math.tan(0.174533)/(2*radius); // hit
                double phi1 = Math.atan2(1,Math.tan(hit1.get_Wire())) ;
                double phi2 = Math.atan2(1,Math.tan(hit.get_Wire())) ;
                double intermediaries = Math.pow(a,3)- Math.pow(b,3);
                double intermediaries1 = Math.sqrt(Math.pow(intermediaries,3)*(32*Math.pow(a-b,3)+9*intermediaries*Math.pow(phi1-phi2,2)));
                double intermediaries2 = -3*Math.pow(a,6)*phi1+6*Math.pow(a,3)*Math.pow(b,3)*phi1-3*Math.pow(b,6)*phi1 + intermediaries1 + 3*Math.pow(a,6)*phi2-6*Math.pow(a,3)*Math.pow(b,3)*phi2+3*Math.pow(b,6)*phi1 ;
                double z = (-4*Math.pow(a,4)+4*Math.pow(a,3)*b+4*Math.pow(b,3)*a-4*Math.pow(b,4)+Math.pow(2,1.0/3)*Math.pow(intermediaries2,2.0/3))/(Math.pow(2,2.0/3)*intermediaries*Math.pow(intermediaries2,1.0/3));
                Cluster clus = new Cluster(radiuscombinehit-radius,(phi1+phi2)/2,z);
                clusters.add(clus);
            }
        }
    }


    public ArrayList<Cluster> getClusters() {
        return clusters;
    }

    public void setClusters(ArrayList<Cluster> clusters) {
        this.clusters = clusters;
    }
}
