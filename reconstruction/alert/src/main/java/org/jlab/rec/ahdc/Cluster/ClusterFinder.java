package org.jlab.rec.ahdc.Cluster;

import org.jlab.rec.ahdc.Hit.Hit;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Cluster are form by 2 hit in layer with different stereo angle to compute z coordinates
 */
public class ClusterFinder {

    public ClusterFinder() {}

    ArrayList<Cluster> _Clusters = new ArrayList<Cluster>();

    public void findClusters(List<Hit> hits){
        Collections.sort(hits);
        for(Hit hit : hits){
            CreateCluster(hit, hits,0,0,1,0,47,32,56,38);
            CreateCluster(hit, hits,1,1,2,0,56,42,72,48);
            CreateCluster(hit, hits,2,1,3,0,72,52,87,58);
            CreateCluster(hit, hits,3,1,4,0,87,62,99,68);
        }
    }

    void NearHitOnSameLayer(Hit hit, List<Hit> hits ,List<Hit> cluster, int layer, int superlayer,int numberofwire){
        for(Hit otherhit : hits) {
            if (otherhit.get_Superlayer() == superlayer && otherhit.get_Layer() == layer){
                for (int i = 1; i <= 3; i++) {
                    if(otherhit.get_Wire() == ((hit.get_Wire() + i) % numberofwire) && !otherhit.get_Used()) {
                        cluster.add(otherhit);
                        otherhit.set_Used(true);
                    }
                    if (otherhit.get_Wire() == ((hit.get_Wire() - i) % numberofwire) && !otherhit.get_Used()) {
                        cluster.add(otherhit);
                        otherhit.set_Used(true);
                    }
                }
            }
        }
    }

    void NearHitOnCombineLayer(Hit hit,List<Hit> hits ,List<Hit> cluster, int combinelayer, int combinesuperlayer,int numberofwire ,int combinenumberofwire){
        for(Hit combinehit : hits){
            if (combinehit.get_Superlayer() == combinesuperlayer && combinehit.get_Layer() == combinelayer && !combinehit.get_Used()) {
                double alphaW_layer = 2*Math.PI/numberofwire;
                double phi_min = alphaW_layer*(hit.get_Wire()-1+0.25);
                double phi_max = phi_min+2*0.349066;
                double alphaW_layercombinehit = 2*Math.PI/ combinenumberofwire;
                double phi_combine = alphaW_layercombinehit*(combinehit.get_Wire()-1+0.25);
                System.out.println("Phi_combinehit : " + phi_combine);
                System.out.println("Phi_max : " +  phi_max);
                System.out.println("Phi_min : " +  phi_min);
                if( phi_combine < phi_max && phi_combine > phi_min){
                    cluster.add(combinehit);
                    combinehit.set_Used(true);
                }
                if(phi_max> 2*Math.PI){
                    if( phi_combine < (phi_max-2*Math.PI) && phi_combine > (phi_min-2*Math.PI)){
                        cluster.add(combinehit);
                        combinehit.set_Used(true);
                    }
                }

            }
        }
    }


    void CreateCluster(Hit hit ,List<Hit> hits ,int superlayer, int layer, int combinesuperlayer, int combinelayer, int num_wire, int radius, int num_wire_combinehit, int radiuscombinehit){
        if(hit.get_Superlayer() == superlayer && hit.get_Layer() == layer && !hit.get_Used()) {
            ArrayList<Hit> cluster = new ArrayList<Hit>();
            cluster.add(hit);
            hit.set_Used(true);

            NearHitOnSameLayer(hit,hits,cluster,layer,superlayer,num_wire);

            NearHitOnCombineLayer(hit,hits,cluster,combinelayer,combinesuperlayer,num_wire,num_wire_combinehit);

            PrintCluster(cluster);

            double phi_hit = 0.0; double phi_combinehit = 0.0; int num_hit = 0 ; int num_combine_hit = 0;

            for (Hit hit1 : cluster) {
                if (hit1.get_Superlayer() == superlayer && hit1.get_Layer() == layer) {
                    phi_hit += (((hit1.get_Wire()-1)*(2*Math.PI)/num_wire)+(1/4)*((2*Math.PI)/num_wire));
                    num_hit++;
                }
                if (hit1.get_Superlayer() == combinesuperlayer && hit1.get_Layer() == combinelayer) {
                    phi_combinehit += ((hit1.get_Wire()-1)*(2*Math.PI)/num_wire_combinehit)+(0.25)*((2*Math.PI)/num_wire_combinehit);
                    num_combine_hit++ ;
                }
            }
            phi_hit /= num_hit;
            phi_combinehit /= num_combine_hit;

            if (!Double.isNaN(phi_hit) && !Double.isNaN(phi_combinehit)) {
                System.out.println("phi_hit : " + phi_hit);
                System.out.println("phi2_combinehit : " + phi_combinehit);
                double minValue = Math.abs(f(0, phi_hit, phi_combinehit));
                double Zfinal = 0;
                for (double Z = 0; Z < 300; Z += 0.1) {
                    double currValue = Math.abs(f(Z, phi_hit, phi_combinehit));
                    if (currValue < minValue) {
                        minValue = currValue;
                        Zfinal = Z;
                    }
                }
                double phi = (phi_hit + phi_combinehit) / 2;
                Cluster clus = new Cluster((double) ((radiuscombinehit + radius) / 2), (phi_hit + phi_combinehit) / 2, Zfinal);
                _Clusters.add(clus);
                double X;
                double Y;
                if(phi_hit<phi_combinehit){
                     X =  -(double) (radiuscombinehit + radius) / 2 * Math.sin(phi);
                     Y =  -(double) (radiuscombinehit + radius) / 2 * Math.cos(phi);}
                else{
                     X =  (double) (radiuscombinehit + radius) / 2 * Math.sin(phi);
                     Y =  (double) (radiuscombinehit + radius) / 2 * Math.cos(phi);
                }
                System.out.println("X : "+ X);
                System.out.println("Y : "+ Y);
                System.out.println("Z : "+ Zfinal);
            }
            cluster.clear();
        }
    }

    private double f(double z, double phi1, double phi2){
        if(phi1+0.349066> 2*Math.PI || phi1<0){return ((phi1+2*Math.asin(z*Math.sin(0.174533)/300))+2*Math.PI )- (phi2+2*Math.asin(z*Math.sin(-0.174533)/300));}
        else if (phi2> 2*Math.PI || phi2-0.349066<0 ){return (phi1+2*Math.asin(z*Math.sin(0.174533)/300)) -( (phi2+2*Math.asin(z*Math.sin(-0.174533)/300))+2*Math.PI);}
        else{return (phi1+2*Math.asin(z*Math.sin(0.174533)/300)) - (phi2+2*Math.asin(z*Math.sin(-0.174533)/300));}

    }

    public ArrayList<Cluster> get_Clusters() {
        return _Clusters;
    }

    public void set_Clusters(ArrayList<Cluster> _Clusters) {
        this._Clusters = _Clusters;
    }

    void PrintCluster(List<Hit> cluster){
        System.out.print("cluster : ");
        for(Hit hit2 : cluster) {
            System.out.print(hit2.get_Wire() + ", " );
        }
        System.out.println(" ");
    }
}
