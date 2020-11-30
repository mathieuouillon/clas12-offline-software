package org.jlab.rec.ahdc.HoughTransform;

import org.jlab.rec.ahdc.Cluster.Cluster;
import Jama.Matrix;
import org.jlab.rec.ahdc.Track.Track;

import java.util.ArrayList;
import java.util.List;

public class HoughTransform {

    private List<Track> _AHDCTracks;

    public HoughTransform(){}

    public void find_tracks(List<Cluster> AHDC_Clusters){
        int matrix_size = 300;
        boolean delete = false;
        ArrayList<Integer> delete_i_max = new ArrayList<>();
        ArrayList<Integer> delete_j_max = new ArrayList<>();

        while(true){
            Matrix B = new Matrix(matrix_size + 1, matrix_size + 1, 0);
            for (Cluster cluster : AHDC_Clusters) {
                double new_u = (cluster.get_U() + 0.06) / 0.12;
                double new_v = (cluster.get_V() + 0.06) / 0.12;
                B.set((int) new_u * matrix_size, (int) new_v * matrix_size, 1);
            }

            ArrayList<Double> theta_ = new ArrayList<>();
            for(double number = 0; number <= 2*Math.PI; number += (2*Math.PI)/1500){
                theta_.add(number);
            }
            Matrix C = new Matrix(matrix_size + 1, matrix_size + 1, 0);
            for(int u = 0; u < matrix_size + 1; u++){
                for(int v = 0; v < matrix_size + 1; v++){
                    if(C.get(u,v) > 0){
                        for(double theta : theta_){
                            double rho = ((double)(u / matrix_size) * 0.06 - 0.03) * Math.cos(theta) + ((double)(v / matrix_size) * 0.06 - 0.03) * Math.sin(theta);
                            double new_rho = (rho + 0.06) / 0.12;
                            double new_theta = (theta - 0.1) / (Math.PI + 0.1 - 0.1);
                            if(rho > -0.0125 && rho < 0.0125){
                                if(rho > -0.0035 && rho < 0.0035){
                                    C.set((int) (new_theta * matrix_size), (int) (new_rho * matrix_size), 0);
                                }
                                else{
                                    C.set((int) (new_theta * matrix_size), (int) (new_rho * matrix_size),
                                            C.get((int) (new_theta * matrix_size), (int) (new_rho * matrix_size)) + 1);
                                }
                            }
                            else{
                                C.set((int) (new_theta * matrix_size), (int) (new_rho * matrix_size), 0);
                            }

                        }
                    }
                }
            }

            if(delete){
                for(int i = 0; i<delete_i_max.size(); i++){
                    for(int ii = -1; ii<2; ii++ ){
                        for(int jj = -1; jj<2; jj++){
                            C.set(delete_i_max.get(i) + ii, delete_j_max.get(i) + jj, 0);
                        }
                    }
                }
            }

            double max_Element = - 1;
            int i_max = - 1;
            int j_max = - 1;

            for(int i = 0; i < matrix_size; i++){
                for(int j = 0; j < matrix_size; j++){
                    if(C.get(i, j) > max_Element){
                        max_Element = C.get(i, j);
                        i_max = i;
                        j_max = j;

                    }
                }
            }

            if(max_Element < 6){break;}

            double theta = (i_max / matrix_size) * Math.PI + 0.1;
            double rho = (j_max / matrix_size) * Math.PI + 0.1;

            if(rho == 0.0){rho = 0.00001;}

            double r = Math.abs(1/(2*rho));
            double a = Math.cos(theta)/(2*rho);
            double b = Math.sin(theta)/(2*rho);

            ArrayList<Cluster> possible_cluster_of_track = new ArrayList<>();
            for(Cluster cluster : AHDC_Clusters){
                double distance = Math.abs(Math.sqrt( (cluster.get_X() - a)*(cluster.get_X() - a) + (cluster.get_Y() - b)*(cluster.get_Y() - b)) - r);
                if(distance < 4){possible_cluster_of_track.add(cluster);}
            }

            if(possible_cluster_of_track.size() > 0){
                double x_0 = possible_cluster_of_track.get(0).get_X();
                double y_0 = possible_cluster_of_track.get(0).get_Y();

                ArrayList<Cluster> cluster_track = new ArrayList<>();
                ArrayList<Cluster> cluster_to_remove = new ArrayList<>();

                for(Cluster other_cluster : possible_cluster_of_track){
                    double distance = Math.sqrt( (other_cluster.get_X() - x_0)*(other_cluster.get_X() - x_0)
                            + (other_cluster.get_Y() - y_0)*(other_cluster.get_Y() - y_0) );
                    if(distance < 50){cluster_track.add(other_cluster);}
                }

                for(int i = 0; i < cluster_track.size(); i++){
                    if(cluster_track.get(i).get_Radius() == cluster_track.get(i+1).get_Radius()){
                        double distance_1 = Math.abs(Math.sqrt( (cluster_track.get(i).get_X() - a)*(cluster_track.get(i).get_X() - a)
                                + (cluster_track.get(i).get_Y() - b)*(cluster_track.get(i).get_Y() - b)) - r);
                        double distance_2 = Math.abs(Math.sqrt( (cluster_track.get(i+1).get_X() - a)*(cluster_track.get(i+1).get_X() - a)
                                + (cluster_track.get(i+1).get_Y() - b)*(cluster_track.get(i+1).get_Y() - b)) - r);
                        if(distance_1<distance_2){cluster_to_remove.add(cluster_track.get(i+1));}
                        else{cluster_to_remove.add(cluster_track.get(i));}
                    }
                }

                ArrayList<Cluster> cluster_to_remove_without_double = new ArrayList<>();
                for(Cluster cluster : cluster_to_remove){
                    if(!cluster_to_remove_without_double.contains(cluster)){cluster_to_remove_without_double.add(cluster);}
                }

                for(Cluster cluster : cluster_to_remove_without_double){cluster_track.remove(cluster);}

                if(cluster_track.size() > 2){
                    _AHDCTracks.add(new Track(cluster_track));
                    for(Cluster cluster : cluster_track){AHDC_Clusters.remove(cluster);}
                }
                else{
                    delete = true;
                    delete_i_max.add(i_max);
                    delete_j_max.add(j_max);
                }
            }
        }
    }

    public List<Track> get_AHDCTracks() {
        return _AHDCTracks;
    }

    public void set_AHDCTracks(List<Track> _AHDCTracks) {
        this._AHDCTracks = _AHDCTracks;
    }
}
