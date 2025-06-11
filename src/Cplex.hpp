#include <ilcplex/ilocplex.h>
#include "MyInstance.hpp"
#include <map>
#include <tuple>
ILOSTLBEGIN

#ifndef MYCPLEX_HPP // Start include guard
#define MYCPLEX_HPP




void CreateModel(IloEnv& env, IloModel& masterModel, MyInstance Inst) {
    std::map<std::tuple<int, int, int, int, int, int>, int> w_index;

    int SizeW = 0;
    vector<string> Varnamew,Varnamef,Varnamex,Varnameu;
    for (int i = 0; i < Inst.node; ++i)
        for (int t = 0; t < Inst.Nt; ++t)
            for (int v = 0; v < Inst.Nv; ++v)
                for (int r = 0; r < Inst.TourVehicle[v]; ++r)
                    for (int k = 0; k < Inst.Nk; ++k)
                        for (int c = 0; c < Inst.Nc; ++c){
                            w_index[std::make_tuple(i, t, v, r, k, c)] = SizeW;
                            ++SizeW;
                            Varnamew.push_back(to_string(i+1)+","+to_string(t+1)+","+to_string(v+1)+","+to_string(r+1)+","+to_string(k+1)+","+to_string(c+1)+"_");
                        }
    
    IloBoolVarArray w(env, SizeW);
    for (int idx = 0; idx < SizeW; ++idx) {
        w[idx] = IloBoolVar(env);
        masterModel.add(w[idx]);
        std::string varName = "w_" + Varnamew[idx];
        w[idx].setName(varName.c_str());
    }

    // Mapping from (i, j, t, v, r) to flat index
    std::map<std::tuple<int, int, int, int, int>, int> x_index;

    int SizeX = 0;
    for (int i = 0; i < Inst.node; ++i) {
        for (int j = 0; j < Inst.node; ++j) {
            if (i == j) continue; // Only i != j
            for (int t = 0; t < Inst.Nt; ++t) {
                for (int v = 0; v < Inst.Nv; ++v) {
                    int nR = Inst.TourVehicle[v];
                    for (int r = 0; r < nR; ++r) {
                        x_index[std::make_tuple(i, j, t, v, r)] = SizeX;
                        ++SizeX;
                        Varnamex.push_back(to_string(i+1)+","+to_string(j+1)+","+to_string(t+1)+","+to_string(v+1)+","+to_string(r+1)+"_");
                    }
                }
            }
        }
    }
    IloBoolVarArray x(env, SizeX);
    for (int idx = 0; idx < SizeX; ++idx) {
        x[idx] = IloBoolVar(env);
        masterModel.add(x[idx]);
        std::string varName = "x_" + Varnamex[idx];
        x[idx].setName(varName.c_str());
    }
    std::map<std::tuple<int, int, int>, int> f_index;
    int SizeF = 0;
    for (int i = 0; i < Inst.Np; ++i)
        for (int k = 0; k < Inst.Nk; ++k)
            for (int c = 0; c < Inst.Nc; ++c){
                f_index[std::make_tuple(i,k,c)] = SizeF;
                ++SizeF;
                Varnamef.push_back(to_string(i+1)+","+to_string(k+1)+","+to_string(c+1)+"_");
    }
    IloBoolVarArray f(env, SizeF);
    for (int idx = 0; idx < SizeF; ++idx) {
        f[idx] = IloBoolVar(env);
        std::string varName = "f_" + Varnamef[idx];
        f[idx].setName(varName.c_str());
        masterModel.add(f[idx]);
    }


    std::map<std::tuple<int, int, int, int>, int> u_index;

    int SizeU = 0;
    for (int i = 0; i < Inst.node; ++i) {
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v = 0; v < Inst.Nv; ++v) {
                if (Inst.StartVehicle[v] == i) continue; // Only if Inst.StartVehicle[v] != i
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    u_index[std::make_tuple(i, t, v, r)] = SizeU;
                    ++SizeU;
                    Varnameu.push_back(to_string(i+1)+","+to_string(t+1)+","+to_string(v+1)+","+to_string(r+1)+"_");
                }
            }
        }
    }
    IloIntVarArray u(env, SizeU);
    for (int idx = 0; idx < SizeU; ++idx) {
        u[idx] = IloIntVar(env, 0, Inst.node); // 0 <= u <= Inst.node
        masterModel.add(u[idx]);
        std::string varName = "u_" + Varnameu[idx];
        u[idx].setName(varName.c_str());
    }

    //----------Objective Function------------
    IloExpr objExpr(env);
    for (int i = 0; i < Inst.node; i++){
        for (int j = 0; j < Inst.node; j++){
            if (i == j) continue;
            for (int t = 0; t < Inst.Nt; t++){
                for (int v = 0; v < Inst.Nv; v++){
                     int nR = Inst.TourVehicle[v];
                    for (int r = 0; r < nR; ++r) {
                        auto x_key = std::make_tuple(i, j, t, v, r);
                        objExpr +=  Inst.dist[i][j] * x[x_index[x_key]];
                    }
                }
            }
        }
    }
    
    // Partie 2 : sum( Inst.dist[i,j+Np+Nh] * f[i,k,j] ...)
    int offset = Inst.Np + Inst.Nh; // Supposant Inst.Np et Inst.Nh définis
    for (int i = 0; i < Inst.Np; ++i) {
        for (int k = 0; k < Inst.Nk; ++k) {
            for (int j = 0; j < Inst.Nc; ++j) {
                int flat_idx = i * Inst.Nk * Inst.Nc + k * Inst.Nc + j;
                objExpr +=  Inst.dist[i][j + offset] * f[flat_idx];
            }
        }
    }

    // Ajout de l'objectif au modèle
    masterModel.add(IloMinimize(env, objExpr));
    objExpr.end();


    //--------------Client is served--------------
    for (int j = 0; j < Inst.Nc; ++j) {
        for (int k = 0; k < Inst.Nk; ++k) {
            if ( Inst.demands[j][k] > 0) { // Seulement si demande > 0
                IloExpr sum_f(env);
                
                // Somme sur tous les i ∈ [0, Inst.Np-1]
                for (int i = 0; i < Inst.Np; ++i) {
                    int flat_idx = i * Inst.Nk * Inst.Nc + k * Inst.Nc + j;
                    sum_f += f[flat_idx];
                }
                
                // Ajout de la contrainte sum_f == 1
                masterModel.add(sum_f == 1);
                sum_f.end();
            }
        }
    }


    //--------------Commande part--------------
    for (int i = 0; i < Inst.Np; ++i) {
        for (int k = 0; k < Inst.Nk; ++k) {
            for (int c = 0; c < Inst.Nc; ++c) {
                IloExpr lhs(env);

                // Première somme : sum(w[i, t, v, r, k, c] *  Inst.Prod_av[i][t] ...)
                for (int t = 0; t < Inst.Nt; ++t) {
                    for (int v : Inst.Vehicles[i]) { // Inst.Vehicles[i] est un std::vector<int>
                        int nR = Inst.TourVehicle[v];
                        for (int r = 0; r < nR; ++r) {
                            auto key = std::make_tuple(i, t, v, r, k, c);
                            lhs += w[w_index[key]] *  Inst.Prod_av[i][t];
                        }
                    }
                }

                // Deuxième somme : sum(w[i, t, v, r, k, c] ...)
                for (int t = 0; t < Inst.Nt; ++t) {
                    for (int h = 0; h < Inst.Nh; ++h) {
                        int idx = h + Inst.Np;
                        for (int v : Inst.Vehicles[idx]) { // Inst.Vehicles[h+Np]
                            int nR = Inst.TourVehicle[v];
                            for (int r = 0; r < nR; ++r) {
                                auto key = std::make_tuple(i, t, v, r, k, c);
                                lhs += w[w_index[key]];
                            }
                        }
                    }
                }

                // Index plat pour f[i,k,c]
                int f_idx = i * Inst.Nk * Inst.Nc + k * Inst.Nc + c;
                masterModel.add(lhs == f[f_idx]);
                lhs.end();
            }
        }
    }



    //--------------Stock OK--------------
    for (int i = 0; i < Inst.Np; ++i) {
        for (int k = 0; k < Inst.Nk; ++k) {
            IloExpr sum_demands(env);
            
            // Calcul de la somme ∑(f[i,k,j] * Inst.demands[j][k])
            for (int j = 0; j < Inst.Nc; ++j) {
                int flat_idx = i * Inst.Nk * Inst.Nc + k * Inst.Nc + j;
                sum_demands += f[flat_idx] * Inst.demands[j][k];
            }
            
            // Ajout de la contrainte  Inst.stocks[i][k] >= somme
            masterModel.add( Inst.stocks[i][k] >= sum_demands);
            sum_demands.end();
        }
    }
    
    //--------------Prod Can't leave if unavailable--------------
    for (int i = 0; i < Inst.Np; ++i) {
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v : Inst.Vehicles[i]) { // Inst.Vehicles[i] is a std::vector<int>
                int nR = Inst.TourVehicle[v];
                for (int k = 0; k < Inst.Nk; ++k) {
                    for (int c = 0; c < Inst.Nc; ++c) {
                        for (int r = 0; r < nR; ++r) {
                            auto key = std::make_tuple(i, t, v, r, k, c);
                            masterModel.add(w[w_index[key]] <=  Inst.Prod_av[i][t]);
                        }
                    }
                }
            }
        }
    }
    

    //--------------Command is delivered--------------
    for (int k = 0; k < Inst.Nk; ++k) {
        for (int c = 0; c < Inst.Nc; ++c) {
            if ( Inst.demands[c][k] > 0) { // Seulement si demande > 0
                IloExpr sum_w(env);
                
                int i = Inst.Cmoins[c];
                for (int t = Inst.DeliWindowsEar[c][k]; t <= Inst.DeliWindowsLat[c][k]; ++t) {
                    for (int v = 0; v < Inst.Nv; ++v) {
                        int nR = Inst.TourVehicle[v];
                        for (int r = 0; r < nR; ++r) {
                            auto key = std::make_tuple(i, t-1, v, r, k, c);
                            sum_w += w[w_index[key]];
                        }
                    }
                }
                masterModel.add(sum_w == 1);
                sum_w.end();
            }
        }
    }

    //--------------Expiration Date--------------
    for (int i = 0; i < Inst.Np; ++i) {
        for (int k = 0; k < Inst.Nk; ++k) {
            for (int c = 0; c < Inst.Nc; ++c) {
                for (int t = 0; t < Inst.Nt; ++t) {
                    for (int tt = 0; tt < Inst.Nt; ++tt) {
                        // Vérifier la condition tt < t OU tt > t +  Inst.Experiation_date[k]
                        if (tt < t || tt > t +  Inst.Experiation_date[k]) {
                            IloExpr sum_w_current(env);
                            IloExpr sum_w_cmoins(env);
                            
                            // Somme pour w[i,t,v,r,k,c]
                            for (int v = 0; v < Inst.Nv; ++v) {
                                int nR = Inst.TourVehicle[v];
                                for (int r = 0; r < nR; ++r) {
                                    auto key = std::make_tuple(i, t, v, r, k, c);
                                    if (w_index.count(key)) {
                                        sum_w_current += w[w_index[key]];
                                    }
                                }
                            }
                            
                            // Somme pour w[Cmoins[c],tt,v,r,k,c]
                            int cmoins_c =Inst.Cmoins[c]; // Ajustement 1-based → 0-based
                            for (int v = 0; v < Inst.Nv; ++v) {
                                int nR = Inst.TourVehicle[v];
                                for (int r = 0; r < nR; ++r) {
                                    auto key = std::make_tuple(cmoins_c, tt, v, r, k, c);
                                    if (w_index.count(key)) {
                                        sum_w_cmoins += w[w_index[key]];
                                    }
                                }
                            }
                            
                            // Ajout de la contrainte
                            masterModel.add(sum_w_current <= 1 - sum_w_cmoins);
                            
                            sum_w_current.end();
                            sum_w_cmoins.end();
                        }
                    }
                }
            }
        }
    }

    //--------------Only one tour--------------
    for (int idx_p = 0; idx_p < Inst.Np+Inst.Nh; ++idx_p) { // Pplus : std::vector<int> (0-based)
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v : Inst.Vehicles[idx_p]) { // Inst.Vehicles[i] : std::vector<int>
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    IloExpr sum_x(env);
                    for (int j = 0; j < Inst.node; ++j) {
                        if (j == idx_p) continue;
                        auto key = std::make_tuple(idx_p, j, t, v, r);
                        sum_x += x[x_index[key]];
                    }
                    masterModel.add(sum_x <= 1);
                    sum_x.end();
                }
            }
        }
    }


    //----------------Visit Once-----------------
    for (int i = 0; i < Inst.node; ++i) {
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v = 0; v < Inst.Nv; ++v) {
                if (i == Inst.StartVehicle[v]) continue; // Inst.StartVehicle[v] : std::vector<int>
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    IloExpr sum_x(env);
                    for (int j = 0; j < Inst.node; ++j) {
                        if (j == i) continue;
                        auto key = std::make_tuple(i, j, t, v, r);
                        sum_x += x[x_index[key]];
                    }
                    masterModel.add(sum_x <= 1);
                    sum_x.end();
                }
            }
        }
    }

    //----------------Flow-----------------
    for (int i = 0; i < Inst.node; ++i) {
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v = 0; v < Inst.Nv; ++v) {
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    IloExpr out_flow(env);
                    IloExpr in_flow(env);
                    
                    // Somme sortante x[i,j,...]
                    for (int j = 0; j < Inst.node; ++j) {
                        if (j == i) continue;
                        auto key_out = std::make_tuple(i, j, t, v, r);
                        if (x_index.count(key_out)) {
                            out_flow += x[x_index[key_out]];
                        }
                    }
                    
                    // Somme entrante x[j,i,...]
                    for (int j = 0; j < Inst.node; ++j) {
                        if (j == i) continue;
                        auto key_in = std::make_tuple(j, i, t, v, r);
                        if (x_index.count(key_in)) {
                            in_flow += x[x_index[key_in]];
                        }
                    }
                    
                    masterModel.add(out_flow == in_flow);
                    out_flow.end();
                    in_flow.end();
                }
            }
        }
    }


    //--------------WorkingHour--------------
    for (int t = 0; t < Inst.Nt; ++t) {
        for (int v = 0; v < Inst.Nv; ++v) {
            IloExpr total_dist(env);
            int nR = Inst.TourVehicle[v];
            
            for (int r = 0; r < nR; ++r) {
                for (int i = 0; i < Inst.node; ++i) {
                    for (int j = 0; j < Inst.node; ++j) {
                        if (i == j) continue;
                        auto key = std::make_tuple(i, j, t, v, r);
                        if (x_index.count(key)) {
                            total_dist += x[x_index[key]] *  Inst.dist[i][j];
                        }
                    }
                }
            }
            
            masterModel.add(total_dist <= Inst.WorkVehicle[v]);
            total_dist.end();
        }
    }
    // ------------------Link w and x---------------
    for (int idx_p = 0; idx_p < Inst.Np+Inst.Nh; ++idx_p) {
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v = 0; v < Inst.Nv; ++v) {
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    for (int k = 0; k < Inst.Nk; ++k) {
                        for (int c = 0; c < Inst.Nc; ++c) {
                            // Vérifier si w[i,t,v,r,k,c] existe
                            auto w_key = std::make_tuple(idx_p, t, v, r, k, c);
                            if (!w_index.count(w_key)) continue;

                            // Calculer sum(x[i,j,t,v,r])
                            IloExpr sum_x(env);
                            for (int j = 0; j < Inst.node; ++j) {
                                if (j == idx_p) continue;
                                auto x_key = std::make_tuple(idx_p, j, t, v, r);
                                if (x_index.count(x_key)) {
                                    sum_x += x[x_index[x_key]];
                                }
                            }
                            
                            // Ajouter la contrainte
                            masterModel.add(w[w_index[w_key]] <= sum_x);
                            sum_x.end();
                        }
                    }
                }
            }
        }
    }

    for (int idx_c = 0; idx_c < (int)Inst.Cmoins.size(); ++idx_c) {
        int i =Inst.Cmoins[idx_c]; //Inst.Cmoins est un std::vector<int> (0-based)
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v = 0; v < Inst.Nv; ++v) {
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    for (int k = 0; k < Inst.Nk; ++k) {
                        for (int c = 0; c < Inst.Nc; ++c) {
                            // Vérifier si w[i,t,v,r,k,c] existe
                            auto w_key = std::make_tuple(i, t, v, r, k, c);
                            if (!w_index.count(w_key)) continue;

                            // Calculer sum(x[j,i,t,v,r])
                            IloExpr sum_x(env);
                            for (int j = 0; j < Inst.node; ++j) {
                                if (j == i) continue;
                                auto x_key = std::make_tuple(j, i, t, v, r);
                                if (x_index.count(x_key)) {
                                    sum_x += x[x_index[x_key]];
                                }
                            }
                            
                            // Ajouter la contrainte
                            masterModel.add(w[w_index[w_key]] <= sum_x);
                            sum_x.end();
                        }
                    }
                }
            }
        }
    }

    // ---------------ProdDrop--------------------
    for (int i = 0; i < Inst.Np; ++i) {
        for (int k = 0; k < Inst.Nk; ++k) {
            for (int c = 0; c < Inst.Nc; ++c) {
                for (int t = 0; t < Inst.Nt; ++t) {
                    for (int v : Inst.Vehicles[i]) { // Véhicules du producteur i
                        int nR = Inst.TourVehicle[v];
                        for (int r = 0; r < nR; ++r) {
                            auto w_key = std::make_tuple(i, t, v, r, k, c);
                            if (!w_index.count(w_key)) continue;

                            // Partie droite de l'équation
                            IloExpr rhs(env);
                            
                            // w[Cmoins[c], t, v, r, k, c]
                            int cmoins_c =Inst.Cmoins[c]; // 
                            auto w_cmoins_key = std::make_tuple(cmoins_c, t, v, r, k, c);
                            if (w_index.count(w_cmoins_key)) {
                                rhs += w[w_index[w_cmoins_key]];
                            }

                            // Somme sur PairHub
                            for (int j = 0; j < Inst.Nh; ++j) {
                                int pairhub_idx = Inst.PairHub[j].second; // PairHub[j,2] en 1-based
                                auto w_hub_key = std::make_tuple(pairhub_idx, t, v, r, k, c);
                                if (w_index.count(w_hub_key)) {
                                    rhs += w[w_index[w_hub_key]];
                                }
                            }

                            masterModel.add(w[w_index[w_key]] == rhs);
                            rhs.end();
                        }
                    }
                }
            }
        }
    }

    // ---------------HubDropAll--------------------
    for (int i = 0; i < Inst.Nh; ++i) {
        for (int k = 0; k < Inst.Nk; ++k) {
            for (int c = 0; c < Inst.Nc; ++c) {
                for (int t = 0; t < Inst.Nt; ++t) {
                    int vehicle_group = i + Inst.Np; // Véhicules du hub i
                    for (int v : Inst.Vehicles[vehicle_group]) {
                        int nR = Inst.TourVehicle[v];
                        for (int r = 0; r < nR; ++r) {
                            IloExpr lhs(env);
                            
                            // Somme sur les producteurs
                            for (int j = 0; j < Inst.Np; ++j) {
                                auto w_key = std::make_tuple(j, t, v, r, k, c);
                                if (w_index.count(w_key)) {
                                    lhs += w[w_index[w_key]];
                                }
                            }

                            // Partie droite w[PairHub[i,2],...]
                            int pairhub_idx = Inst.PairHub[i].second; // PairHub[i,2] en 1-based
                            auto w_ph_key = std::make_tuple(pairhub_idx, t, v, r, k, c);
                            if (w_index.count(w_ph_key)) {
                                masterModel.add(lhs == w[w_index[w_ph_key]]);
                            }
                            
                            lhs.end();
                        }
                    }
                }
            }
        }
    }



    // ---------------HubDeliAll--------------------
    for (int i = 0; i < Inst.Nh; ++i) {
        for (int k = 0; k <Inst. Nk; ++k) {
            for (int c = 0; c < Inst.Nc; ++c) {
                for (int t = 0; t < Inst.Nt; ++t) {
                    int vehicle_group = i + Inst.Np;  // Hub vehicles start after producers
                    for (int v : Inst.Vehicles[vehicle_group]) {
                        int nR = Inst.TourVehicle[v];
                        for (int r = 0; r < nR; ++r) {
                            // Left side: w[PairHub[i,1],...]
                            int ph1 = Inst.PairHub[i].first;  // Convert 1-based to 0-based
                            auto left_key = std::make_tuple(ph1, t, v, r, k, c);
                            
                            // Right side: w[Cmoins[c],...]
                            int cmoins_idx = Inst.Cmoins[c];  // Convert 1-based to 0-based
                            auto right_key = std::make_tuple(cmoins_idx, t, v, r, k, c);

                            // Check if both variables exist
                            if (w_index.count(left_key) && w_index.count(right_key)) {
                                masterModel.add(w[w_index[left_key]] == w[w_index[right_key]]
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    //-----------------Hub Deliver next Period-----------------
    for (int i = 0; i < Inst.Nh; ++i) {
        for (int t = 0; t < Inst.Nt; ++t) {
            int vehicle_group = i + Inst.Np; // Véhicules du hub i
            for (int v : Inst.Vehicles[vehicle_group]) {
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    for (int k = 0; k < Inst.Nk; ++k) {
                        for (int c = 0; c < Inst.Nc; ++c) {
                            // Partie gauche w[PairHub[i,1],...]
                            int ph1 = Inst.PairHub[i].first; // 
                            auto w_key = std::make_tuple(ph1, t, v, r, k, c);

                            // Partie droite : somme sur tt < t
                            IloExpr rhs(env);
                            for (int tt = 0; tt < t; ++tt) {
                                // Producteurs (j ∈ [0, Inst.Np-1])
                                for (int j = 0; j < Inst.Np; ++j) {
                                    for (int vv : Inst.Vehicles[j]) {
                                        int nRR = Inst.TourVehicle[vv];
                                        for (int rr = 0; rr < nRR; ++rr) {
                                            auto key = std::make_tuple(Inst.PairHub[i].second, tt, vv, rr, k, c);
                                            if (w_index.count(key)) rhs += w[w_index[key]];
                                        }
                                    }
                                }
                                // Véhicules du hub
                                for (int vv : Inst.Vehicles[vehicle_group]) {
                                    int nRR = Inst.TourVehicle[vv];
                                    for (int rr = 0; rr < nRR; ++rr) {
                                        auto key = std::make_tuple(Inst.PairHub[i].second, tt, vv, rr, k, c);
                                        if (w_index.count(key)) rhs += w[w_index[key]];
                                    }
                                }
                            }
                            if(i==0 && t==2 && v==3 && r==3 && k==0 && c==9)
                                cout<<w[w_index[w_key]]<<" <= "<<rhs<<endl;
                            masterModel.add(w[w_index[w_key]] <= rhs);
                            rhs.end();
                        }
                    }
                }
            }
        }
    }

    //----------------Capacity OK-------------------
    for (int t = 0; t < Inst.Nt; ++t) {
        for (int v = 0; v < Inst.Nv; ++v) {
            int nR = Inst.TourVehicle[v];
            for (int r = 0; r < nR; ++r) {
                IloExpr total_load(env);
                
                for (int k = 0; k < Inst.Nk; ++k) {
                    for (int c = 0; c < Inst.Nc; ++c) {
                        for (int idx_p=0; idx_p<Inst.Np+Inst.Nh;idx_p++) { // Pplus : std::vector<int>
                            auto key = std::make_tuple(idx_p, t, v, r, k, c);
                            if (w_index.count(key)) {
                                total_load += w[w_index[key]] * Inst.demands[c][k] * Inst.Psize[k];
                            }
                        }
                    }
                }
                
                masterModel.add(total_load <= Inst.CapaVehicle[v]);
                total_load.end();
            }
        }
    }


    //------------------- DeliveryOrPick-----------------
    for (int j = 0; j < Inst.Nh; ++j) {
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v = 0; v < Inst.Nv; ++v) {
                if (Inst.PairHub[j].first == Inst.StartVehicle[v]) continue; // 
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    IloExpr sum_x(env);
                    for (int idx_p=0;idx_p<Inst.Np+Inst.Nh;idx_p++) { // Pplus : std::vector<int>
                        if (idx_p == Inst.StartVehicle[v]) continue;
                        auto key = std::make_tuple(idx_p, Inst.PairHub[j].second, t, v, r);
                        if (x_index.count(key)) {
                            sum_x += x[x_index[key]];
                        }
                    }
                    masterModel.add(sum_x == 0);
                    sum_x.end();
                }
            }
        }
    }

    for (int j = 0; j < Inst.Nh; ++j) {
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v = 0; v < Inst.Nv; ++v) {
                if (Inst.PairHub[j].first == Inst.StartVehicle[v]) continue;
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    IloExpr sum_x(env);
                    for (int idx_p=0;idx_p<Inst.Np+Inst.Nh;idx_p++) {
                        if (idx_p == Inst.StartVehicle[v]) continue;
                        auto key = std::make_tuple(Inst.PairHub[j].second, idx_p, t, v, r);
                        if (x_index.count(key)) {
                            sum_x += x[x_index[key]];
                        }
                    }
                    masterModel.add(sum_x == 0);
                    sum_x.end();
                }
            }
        }
    }

    for (int j = 0; j < Inst.Nc; ++j) {
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v = 0; v < Inst.Nv; ++v) {
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    IloExpr sum_x(env);
                    int client_node = j + Inst.Np + Inst.Nh; // Conversion 1-based → 0-based
                    for (int idx_p=0;idx_p<Inst.Np+Inst.Nh;idx_p++) {
                        if (idx_p == Inst.StartVehicle[v]) continue;
                        auto key = std::make_tuple(idx_p, client_node, t, v, r);
                        if (x_index.count(key)) {
                            sum_x += x[x_index[key]];
                        }
                    }
                    masterModel.add(sum_x == 0);
                    sum_x.end();
                }
            }
        }
    }

    for (int j = 0; j < Inst.Nc; ++j) {
        for (int t = 0; t < Inst.Nt; ++t) {
            for (int v = 0; v < Inst.Nv; ++v) {
                int nR = Inst.TourVehicle[v];
                for (int r = 0; r < nR; ++r) {
                    IloExpr sum_x(env);
                    int client_node = j + Inst.Np + Inst.Nh; // Conversion 1-based → 0-based
                    for (int idx_p=0;idx_p<Inst.Np+Inst.Nh;idx_p++) {
                        if (idx_p == Inst.StartVehicle[v]) continue;
                        auto key = std::make_tuple(client_node, idx_p, t, v, r);
                        if (x_index.count(key)) {
                            sum_x += x[x_index[key]];
                        }
                    }
                    masterModel.add(sum_x == 0);
                    sum_x.end();
                }
            }
        }
    }
    // --------------- TooFarPoint---------------
    for (int i = 0; i < Inst.Np; ++i) {
        for (int j = 0; j < Inst.node; ++j) {
            if (Inst.dist[i][j] < Inst.ProdMaxRange[i]) continue;
            for (int v : Inst.Vehicles[i]) {
                for (int t = 0; t < Inst.Nt; ++t) {
                    IloExpr sum_w(env);
                    int nR = Inst.TourVehicle[v];
                    
                    for (int r = 0; r < nR; ++r) {
                        for (int k = 0; k < Inst.Nk; ++k) {
                            for (int c = 0; c < Inst.Nc; ++c) {
                                auto key = std::make_tuple(j, t, v, r, k, c);
                                if (w_index.count(key)) {
                                    sum_w += w[w_index[key]];
                                }
                            }
                        }
                    }
                    masterModel.add(sum_w == 0);
                    sum_w.end();
                }
            }
        }
    }

    //----------------flowhub---------------
    for (int i = 0; i < Inst.Nh; ++i) {
        int vehicle_group = i + Inst.Np;
        for (int v : Inst.Vehicles[vehicle_group]) {
            int nR = Inst.TourVehicle[v];
            for (int r = 0; r < nR; ++r) {
                for (int t = 0; t < Inst.Nt; ++t) {
                    int ph1 = Inst.PairHub[i].first ; // Conversion 1-based → 0-based
                    int ph2 = Inst.PairHub[i].second;
                    
                    // Somme x[ph1,j,...]
                    IloExpr sum_x(env);
                    for (int j = 0; j < Inst.node; ++j) {
                        if (j == ph1) continue;
                        auto key = std::make_tuple(ph1, j, t, v, r);
                        if (x_index.count(key)) {
                            sum_x += x[x_index[key]];
                        }
                    }
                    
                    // x[ph2,ph1,...]
                    auto key_ph = std::make_tuple(ph2, ph1, t, v, r);
                    if (x_index.count(key_ph)) {
                        masterModel.add(sum_x == x[x_index[key_ph]]);
                    }
                    
                    sum_x.end();
                }
            }
        }
    }

    // --------------- subtour---------------
    for (int v = 0; v < Inst.Nv; ++v) {
        int start_v = Inst.StartVehicle[v];
        for (int t = 0; t < Inst.Nt; ++t) {
            int nR = Inst.TourVehicle[v];
            for (int r = 0; r < nR; ++r) {
                for (int i = 0; i < Inst.node; ++i) {
                    if (i == start_v) continue;
                    for (int j = 0; j < Inst.node; ++j) {
                        if (j == start_v || i == j) continue;
                        
                        // Accès aux variables
                        auto x_ij_key = std::make_tuple(i, j, t, v, r);
                        auto x_ji_key = std::make_tuple(j, i, t, v, r);
                        auto u_i_key = std::make_tuple(i, t, v, r);
                        auto u_j_key = std::make_tuple(j, t, v, r);
                        
                        if (x_index.count(x_ij_key) && x_index.count(x_ji_key) && 
                            u_index.count(u_i_key) && u_index.count(u_j_key)) {
                            
                            IloExpr expr(env);
                            expr += u[u_index[u_i_key]] - u[u_index[u_j_key]] 
                                + Inst.node * x[x_index[x_ij_key]] 
                                + (Inst.node - 2) * x[x_index[x_ji_key]];
                            
                            masterModel.add(expr <= Inst.node - 1);
                            expr.end();
                        }
                    }
                }
            }
        }
    }
}

void CPLEXOPTIMIZE(MyInstance Inst,int ub){
    IloEnv env;	
    IloModel MasterModel(env);
    cout<<"CONSTRUCT MODEL"<<endl;
    CreateModel(env,MasterModel,Inst);
    IloCplex MasterCplex(MasterModel);
    MasterCplex.setParam(IloCplex::Param::Threads, 1);
    MasterCplex.setParam(IloCplex::Param::TimeLimit,3600);
    ub=1029;
    MasterCplex.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, ub);
    cout<<"SolveModel"<<endl;
    MasterCplex.exportModel("test.lp");
    MasterCplex.solve();
    int Opt=-1;
    string Stat;
    if (MasterCplex.getStatus() == IloAlgorithm::Optimal){
        Stat="OPT";
        Opt=ceil(MasterCplex.getObjValue()-0.000001);
    }
    else if (MasterCplex.getStatus() == IloAlgorithm::Optimal){ 
        Stat="FEAS";
        Opt=ceil(MasterCplex.getObjValue()-0.000001);
    }
    else if (MasterCplex.getStatus() == IloAlgorithm::Infeasible){ 
        Stat="UNF";
        Opt=ceil(MasterCplex.getObjValue()-0.000001);
    }else {
        Stat="UNK";
    }
    cout<<"Time "<<MasterCplex.getTime()<<endl;
    cout<<"Status "<<Stat<<endl;
    cout<<"OPT "<<Opt<<endl;
}

#endif