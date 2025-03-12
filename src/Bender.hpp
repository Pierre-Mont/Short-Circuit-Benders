#include <ilcplex/ilocplex.h>
#include <string>
#include <chrono>
#include <numeric>

#ifdef USE_BAP
	#include "Bapcod.hpp"
#else
	#include "MyInstance.hpp"
#endif
ILOSTLBEGIN

#ifndef MYBENDER_HPP // Start include guard
#define MYBENDER_HPP
template <typename T1, typename T2, typename T3>
class Triplet {
public:
    T1 first;
    T2 second;
    T3 third;

    Triplet(T1 a, T2 b, T3 c) : first(a), second(b), third(c) {}
};


template <typename T>
ostream &operator<<(std::ostream &os, const std::vector<T> &input)
{
	os <<"[";
    for (auto const &i: input) {
        os << i << " ";
    }
	os<< "]";
    return os;
}

template <typename T>
ostream &operator<<(std::ostream &os, const std::vector<vector<T>> &input)
{
	os<< "[";
    for (int i=0; i<(int)input.size();i++) {
        os << input[i] << " ";
    }
	os<< "]";
    return os;
    }
    


void createMasterModel(IloEnv& env, IloModel& masterModel, MyInstance Inst, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>>& w, IloArray<IloArray<IloNumVar>>& sigma,IloArray<IloArray<IloBoolVarArray>>& f,IloArray<IloArray<IloBoolVarArray>>& y, IloArray<IloArray<IloNumVarArray>>& fr, IloArray<IloArray<IloNumVarArray>>& yr ) {
   	for (int i = 0; i < Inst.node; ++i) {
		w[i] = IloArray<IloArray<IloArray<IloBoolVarArray>>>(env, Inst.Nt);
		for (int t = 0; t < Inst.Nt; ++t) {
			//If the node is a producer then check if he is available
			w[i][t] = IloArray<IloArray<IloBoolVarArray>>(env, Inst.Nv);
			for (int v = 0; v < Inst.Nv; ++v) {
				//If the node is a producer then only himself and hub can pick.
				w[i][t][v] = IloArray<IloBoolVarArray>(env, Inst.Nk);
				for (int k = 0; k < Inst.Nk; ++k) {
					w[i][t][v][k] = IloBoolVarArray(env, Inst.Nc);
					for (int c = 0; c < Inst.Nc; ++c) {
						//Alawys create a variable if hub or producer. Create one variable for the client. 
						//if(i<Inst.Np+Inst.Nh || i > Inst.Np + Inst.Nh + Inst.Nc -1 || c==i-Inst.Np-Inst.Nh ){
							ostringstream varName;
							varName << "w_" << i << "_" << t << "_" << v << "_"<<k <<"_"<<c;
							w[i][t][v][k][c] = IloBoolVar(env, varName.str().c_str());
						//}
					}
				}
			}
		}
	}
	if(Inst.FReal==0){
		for (int i = 0; i < Inst.Np; ++i) {
			f[i] = IloArray<IloBoolVarArray>(env, Inst.Nk);
			for (int k = 0; k < Inst.Nk; ++k) {
				f[i][k] = IloBoolVarArray(env, Inst.Nc);
				for (int c = 0; c < Inst.Nc; ++c) {
					if(Inst.demands[c][k]>0){
						ostringstream varName;
						varName << "f_" << i <<"_"<<k <<"_"<<c;
						f[i][k][c] = IloBoolVar(env, varName.str().c_str());
					}
				}
			}
		}
	}else{
		for (int i = 0; i < Inst.Np; ++i) {
			fr[i] = IloArray<IloNumVarArray>(env, Inst.Nk);
			for (int k = 0; k < Inst.Nk; ++k) {
				fr[i][k] = IloNumVarArray(env, Inst.Nc);
				for (int c = 0; c < Inst.Nc; ++c) {
					if(Inst.demands[c][k]>0){
						ostringstream varName;
						varName << "f_" << i <<"_"<<k <<"_"<<c;
						fr[i][k][c] = IloNumVar(env, 0.0,1.0,varName.str().c_str());
					}
				}
				
			}
		}
	}
	for (int t = 0; t < Inst.Nt; ++t) {
		sigma[t] = IloArray<IloNumVar>(env, Inst.Nv);
		for (int v = 0; v < Inst.Nv; ++v) {
			ostringstream varName;
			varName << "sig_" << t <<"_"<<v;
			sigma[t][v] = IloNumVar(env, 0.0, Inst.TourVehicle[v]*Inst.WorkVehicle[v],varName.str().c_str());
		}		
	}
	// Define the objective function
	IloExpr objective(env);
	if(Inst.NoObj==0){
		for (int i = 0; i < Inst.Np; ++i) {
			for (int k = 0; k < Inst.Nk; ++k) {
				if(Inst.stocks[i][k]>0){
					for (int j = 0; j < Inst.Nc; ++j) {	
						if(Inst.demands[j][k]>0){
							if(Inst.FReal==0)
								objective += Inst.ImortanceObj* Inst.dist[i][j+Inst.Np+Inst.Nh] * f[i][k][j];
							else
								objective += Inst.ImortanceObj*Inst.dist[i][j+Inst.Np+Inst.Nh] * fr[i][k][j];
						}
					}
				}
			}
		}
	}

	for (int t = 0; t < Inst.Nt; ++t) {
		for (int v = 0; v < Inst.Nv; ++v) {
			objective += sigma[t][v];
		}
	}
	masterModel.add(IloMinimize(env, objective));
	objective.end();

	 // Constraints
	 //Clients are served
	for (int j = 0; j < Inst.Nc; ++j) {
		for (int k = 0; k < Inst.Nk; ++k) {
			if (Inst.demands[j][k] > 0) {
				IloExpr client_service(env);
				for (int i = 0; i < Inst.Np; ++i) {
					if(Inst.stocks[i][k]>0){
						if(Inst.FReal==0)
							client_service += f[i][k][j];
						else
							client_service += fr[i][k][j];
					}
				}
				masterModel.add(client_service == 1);
				client_service.end();
			}
		}
	}
	// Stocks are valid
	for (int i = 0; i < Inst.Np; ++i) {
		for (int k = 0; k < Inst.Nk; ++k) {		
			IloExpr stock_ok(env);
			for (int j = 0; j < Inst.Nc; ++j) {
				if(Inst.demands[j][k]>0){
					if(Inst.FReal==0)
						stock_ok += f[i][k][j] * Inst.demands[j][k];
					else
						stock_ok += fr[i][k][j] * Inst.demands[j][k];
				}
			}
			masterModel.add(Inst.stocks[i][k] >= stock_ok);
			stock_ok.end();		
		}
	}
	//Command leave producers
	for (int c = 0; c < Inst.Nc; ++c) {
		for (int k = 0; k < Inst.Nk; ++k) {
			if(Inst.demands[c][k]>0){
				for (int i = 0; i < Inst.Np; ++i) {
					IloExpr command_leave(env);
					for (int t = 0; t < Inst.Nt; ++t) {
						if(Inst.Prod_av[i][t]){
							for (int v : Inst.Vehicles[i]) {
								command_leave += w[i][t][v][k][c];
							}
						}
					}
					for (int t = 0; t < Inst.Nt; ++t) {
						for (int h = 0; h < Inst.Nh; ++h) {
							for (int v : Inst.Vehicles[h + Inst.Np]) {
								command_leave += w[i][t][v][k][c];
							}
						}
					}
					if(Inst.FReal==0)
						masterModel.add(command_leave == f[i][k][c]);
					else
						masterModel.add(command_leave == fr[i][k][c]);
					command_leave.end();
				}
			}
		}
	}
	for (int i = 0; i < Inst.Np; ++i) {
		for (int t = 0; t < Inst.Nt; ++t) {
			for (int v : Inst.Vehicles[i]) {
				for (int k = 0; k < Inst.Nk; ++k) {
					for (int c = 0; c < Inst.Nc; ++c) {
						if(Inst.Prod_av[i][t]==0)
							w[i][t][v][k][c].setUB(0);
						//masterModel.add(w[i][t][v][k][c] <= Inst.Prod_av[i][t]);
					}
				}
			}
		}
	}
	//command arrive at destination during time windows 
	for (int k = 0; k < Inst.Nk; ++k) {
		for (int c = 0; c < Inst.Nc; ++c) {
			if (Inst.demands[c][k] > 0) {
				IloExpr command_arrive(env);
				for (int t = Inst.DeliWindowsEar[c][k]; t <= Inst.DeliWindowsLat[c][k]; ++t) {
					for (int v = 0; v < Inst.Nv; ++v) {
						if(Inst.StartVehicle[v]>= Inst.Np || (Inst.Prod_av[Inst.StartVehicle[v]][t-1] && Inst.stocks[Inst.StartVehicle[v]][k]>0))
							command_arrive += w[Inst.Cmoins[c]][t-1][v][k][c] * Inst.Client_av[c][t-1];
					}
				}
				masterModel.add(command_arrive == 1);
				command_arrive.end();
			}
		}
	}
	//Expiration date
	for (int i = 0; i < Inst.Np; ++i) {
		for (int k = 0; k < Inst.Nk; ++k) {
			for (int c = 0; c < Inst.Nc; ++c) {
				if(Inst.demands[c][k]>0){
					for (int t = 0; t < Inst.Nt; ++t) {
						for (int tt = 0; tt < Inst.Nt; ++tt) {
							if (tt < t || tt > t + Inst.Experiation_date[k]) {
								IloExpr expi(env);
								for (int v = 0; v < Inst.Nv; ++v) {
									if(Inst.StartVehicle[v]>= Inst.Np || (Inst.StartVehicle[v]==i && Inst.Prod_av[i][t]))
										expi += w[i][t][v][k][c];
								}
								for (int v = 0; v < Inst.Nv; ++v) {
									if(Inst.StartVehicle[v]>= Inst.Np || (Inst.StartVehicle[v]==i && Inst.Prod_av[i][tt]))
										expi += w[Inst.Cmoins[c]][tt][v][k][c];
								}
								masterModel.add(expi <= 1);
								expi.end();
							}
						}
						
					}
				}
			}
		}
	}
	if(Inst.NewForm==0){
	//Prod must drop 
		IloExpr prod_drop(env);
		for (int i =0;i< Inst.Np; ++i) {
			for (int k = 0; k < Inst.Nk; ++k) {
				for (int c = 0; c < Inst.Nc; ++c) {
					if(Inst.demands[c][k]>0){
						for (int t = 0; t < Inst.Nt; ++t) {
							for (int v : Inst.Vehicles[i]) {
								prod_drop.clear();
								prod_drop += w[Inst.Cmoins[c]][t][v][k][c];
								for (int j = 0; j < Inst.Nh; ++j) {
									if (j + Inst.Np != i) {
										prod_drop += w[Inst.PairHub[j].second][t][v][k][c];
									}
								}
								masterModel.add(w[i][t][v][k][c] == prod_drop);
							}
						}
					}
				}
			}
		}	
		for (int i = 0; i < Inst.Nh; ++i) {
			for (int k = 0; k < Inst.Nk; ++k) {
				for (int c = 0; c < Inst.Nc; ++c) {
					if(Inst.demands[c][k]>0){
						for (int t = 0; t < Inst.Nt; ++t) {
							for (int v : Inst.Vehicles[i+Inst.Np]) {
								prod_drop.clear();
								prod_drop += w[Inst.Cmoins[c]][t][v][k][c];
								masterModel.add(w[i+Inst.Np][t][v][k][c]  == prod_drop);
							}
						}
					}
				}
			}
		}	
		prod_drop.end();
		//Hub must drop 
		for (int i = 0; i < Inst.Nh; ++i) {
			for (int k = 0; k < Inst.Nk; ++k) {
				for (int c = 0; c < Inst.Nc; ++c) {
					if(Inst.demands[c][k]>0){
						for (int t = 0; t < Inst.Nt; ++t) {
							for (int v : Inst.Vehicles[i + Inst.Np]) {
								IloExpr hub_drop_all(env);
								for (int j =0; j < Inst.Np;j++) {	
									hub_drop_all += w[j][t][v][k][c];
								}
								masterModel.add(hub_drop_all == w[Inst.PairHub[i].second][t][v][k][c]);
								hub_drop_all.end();
							}
						}
					}
				}
			}
		}
	}else{
		//Vehicle pick and drop
		IloExpr prod_drop(env);
		for (int i : Inst.Pplus) {
			for (int k = 0; k < Inst.Nk; ++k) {
				for (int c = 0; c < Inst.Nc; ++c) {
					if(Inst.demands[c][k]>0){
						for (int t = 0; t < Inst.Nt; ++t) {
							for (int v : Inst.Vehicles[i]) {
								prod_drop.clear();
								prod_drop += w[Inst.Cmoins[c]][t][v][k][c];
								for (int j = 0; j < Inst.Nh; ++j) {	
									prod_drop += w[Inst.PairHub[j].second][t][v][k][c];
									prod_drop -= w[Inst.PairHub[j].first][t][v][k][c];
								}
								for (int j = 0; j < Inst.Np; ++j) {
									prod_drop -= w[j][t][v][k][c];
								}
								masterModel.add(0 == prod_drop);
							}
						}
					}
				}
			}
		}	
		prod_drop.end();
	}
	//if command arrives at hub it must leave next day
	for (int i = 0; i < Inst.Nh; ++i) {
		for (int t = 0; t < Inst.Nt; ++t) {
			for (int v = 0; v < Inst.Nv; ++v) {
				for (int k = 0; k < Inst.Nk; ++k) {
					for (int c = 0; c < Inst.Nc; ++c) {
						if(Inst.demands[c][k]>0){
							IloExpr expr(env);
							for (int tt = 0; tt < t; ++tt) {
								for (int vv = 0; vv < Inst.Nv; ++vv) {
									if(Inst.StartVehicle[vv]<Inst.Np || Inst.StartVehicle[vv]==i+Inst.Np)
										expr += w[Inst.PairHub[i].second][tt][vv][k][c];
								}
							}
							masterModel.add(w[Inst.PairHub[i].first][t][v][k][c] <= expr);
							expr.end();
						}
					}
				}
			}
		}
	}
	
	//Capa Prod
	for (int t = 0; t < Inst.Nt; ++t) {
		for (int i =0; i< Inst.Np;++i) {
			for (int v: Inst.Vehicles[i]) {
				IloExpr expr(env);
				for (int k = 0; k < Inst.Nk; ++k) {
					for (int c = 0; c < Inst.Nc; ++c) {
						expr += w[i][t][v][k][c] * Inst.demands[c][k] * Inst.Psize[k];
					}
				}
				masterModel.add(expr <= Inst.CapaVehicle[v]*Inst.TourVehicle[v]);
				expr.end();
			}
		}
	}
	//Capa Hub
	/*for (int t = 0; t < Inst.Nt; ++t) {
		for (int i =0; i< Inst.Nh;++i) {
			for (int v: Inst.Vehicles[i+Inst.Np]) {
				IloExpr expr(env);
				for (int j : Inst.Pplus){
					for (int k = 0; k < Inst.Nk; ++k) {
						for (int c = 0; c < Inst.Nc; ++c) {
							expr += w[j][t][v][k][c] * Inst.demands[c][k] * Inst.Psize[k];
						}
					}
				}
				for (int j : Inst.Cmoins){
					if(j!=Inst.PairHub[i].second){
						for (int k = 0; k < Inst.Nk; ++k) {
							for (int c = 0; c < Inst.Nc; ++c) {
								expr += w[j][t][v][k][c] * Inst.demands[c][k]* Inst.Psize[k];
							}
						}
					}
				}
				masterModel.add(expr <= Inst.CapaVehicle[v]*Inst.TourVehicle[v]);
				expr.end();
			}
		}
	}*/

	
	int sumw;
	for (int t = 0; t < Inst.Nt; ++t) {
		for (int i =0; i< Inst.Nh;++i) {
			for (int v: Inst.Vehicles[i+Inst.Np]) {
				for (int j : Inst.Pplus){
					if(j!=Inst.StartVehicle[v]){
						IloExpr expr(env);
						sumw=0;
						for (int k = 0; k < Inst.Nk; ++k) {
							for (int c = 0; c < Inst.Nc; ++c) {
								sumw+=Inst.demands[c][k]* Inst.Psize[k];
								expr += w[j][t][v][k][c] * Inst.demands[c][k]* Inst.Psize[k];
							}
						}
						if(sumw > Inst.CapaVehicle[v])
							masterModel.add(expr <= Inst.CapaVehicle[v]);
						expr.end();
					}
				}
				for (int j : Inst.Cmoins){
					if(j!=Inst.PairHub[i].second){
						IloExpr expr(env);
						sumw=0;
						for (int k = 0; k < Inst.Nk; ++k) {
							for (int c = 0; c < Inst.Nc; ++c) {
								sumw+=Inst.demands[c][k]* Inst.Psize[k];
								expr += w[j][t][v][k][c] * Inst.demands[c][k]* Inst.Psize[k];
							}
						}
						if(sumw > Inst.CapaVehicle[v])
							masterModel.add(expr <= Inst.CapaVehicle[v]);
						expr.end();
					}
				}
				
			}
		}
	}
	
	

	//Prod radius
	for (int t = 0; t < Inst.Nt; ++t) {
		for (int i = 0; i < Inst.Np; ++i) {
			for (int v: Inst.Vehicles[i]) {
				for (int j : Inst.Cmoins) {
					if(Inst.dist[i][j] > min(42.0,Inst.WorkProd/2.0)){
						for (int k = 0; k < Inst.Nk; k++){
							for (int c = 0; c < Inst.Nc; c++){
								if (Inst.demands[c][k]>0) {
									w[j][t][v][k][c].setUB(0);
									//masterModel.add( w[j][t][v][k][c] == 0);
								}
							}
						}
					}
				}
			}
		}
	}
	for (int t = 0; t < Inst.Nt; ++t) {
		for (int h = 0; h < Inst.Nh; ++h) {
			for (int v: Inst.Vehicles[h+Inst.Np]) {
				for (int j =0;j<Inst.node; j++) {
					if(Inst.dist[h+Inst.Np][j] > (Inst.WorkVehicle[v] / 2.0)){
						for (int k = 0; k < Inst.Nk; k++){
							for (int c = 0; c < Inst.Nc; c++){
								//cout<<w[j][t][v][k][c]<<endl;
								//masterModel.add( w[j][t][v][k][c] == 0);
								w[j][t][v][k][c].setUB(0);
								
							}
						}
					}
				}
			}
		}
	}
	if(Inst.MoreZero==1){

		// Constraint 3: ProdCantPick
		for (int t = 0; t < Inst.Nt; ++t) {
			for (int i = 0; i < Inst.Np; ++i) {
				for (int v: Inst.Vehicles[i]) {
					for (int j : Inst.Pplus) {
						if (Inst.Pplus[j] != i) {
							for (int k = 0; k < Inst.Nk; ++k) {
								for (int c = 0; c < Inst.Nc; ++c) {
									
									w[j][t][v][k][c].setUB(0);
									//masterModel.add( w[j][t][v][k][c] == 0);
								}
							}
						}
					}
				}
			}
		}

		// Constraint 3: HubCantPickHub
		for (int t = 0; t < Inst.Nt; ++t) {
			for (int i = 0; i < Inst.Nh; ++i) {
				for (int v: Inst.Vehicles[i+Inst.Np]) {
					for (int j=0;j < Inst.Nh; ++j) {
						if (j != i) {
							for (int k = 0; k < Inst.Nk; ++k) {
								for (int c = 0; c < Inst.Nc; ++c) {
									w[Inst.PairHub[j].first][t][v][k][c].setUB(0);
									w[Inst.PairHub[j].second][t][v][k][c].setUB(0);
									//masterModel.add( w[Inst.PairHub[j].first][t][v][k][c] == 0);
									//masterModel.add( w[Inst.PairHub[j].second][t][v][k][c] == 0);
									
								}
							}
						}
					}
				}
			}
		}
		//Cant deliver customer outside delivery windows
		for (int t = 0; t < Inst.Nt; ++t) {
			for (int v = 0; v < Inst.Nv; ++v) {			
				for (int c =0; c < Inst.Nc; ++c) {
					for (int k =0; k < Inst.Nk; ++k) {
						if (Inst.demands[c][k]==0 || !Inst.Client_av[c][t] || Inst.DeliWindowsEar[c][k] - 1 > t || Inst.DeliWindowsLat[c][k] -1 < t ) {
							//masterModel.add( w[Inst.Cmoins[c]][t][v][k][c] == 0);
							w[Inst.Cmoins[c]][t][v][k][c].setUB(0);
						}
					}
				}
			}
		}

		//Cant deliver hub too late or too early
		for (int t = 0; t < Inst.Nt; ++t) {
			for (int v = 0; v < Inst.Nv; ++v) {			
				for (int k =0; k < Inst.Nk; ++k) {
					for (int c =0; c < Inst.Nc; ++c) {
						if(Inst.demands[c][k]){
							for (int h =0; h < Inst.Nh; ++h) {
								if(Inst.DeliWindowsLat[c][k]-1 < t || Inst.DeliWindowsEar[c][k]-Inst.Experiation_date[k] -1 > t){
									//masterModel.add( w[Inst.PairHub[h].second][t][v][k][c] == 0);
									w[Inst.PairHub[h].second][t][v][k][c].setUB(0);
								}
							}
						}
					}
				}
			}
		}

		//Producer stock
		for (int i = 0; i < Inst.Np; ++i) {
			for (int c = 0; c < Inst.Nc; ++c) {			
				for (int k =0; k < Inst.Nk; ++k) {
					if(Inst.stocks[i][k] < Inst.demands[c][k]){
						if(Inst.FReal==0){
							//masterModel.add( f[i][k][c] == 0);
							f[i][k][c].setUB(0);
						}
						else{
							//masterModel.add( fr[i][k][c] == 0);
							fr[i][k][c].setUB(0);
						}
					}
				}
			}
		}
		//hub can't go too fare.
		for (int t = 0; t < Inst.Nt; ++t) {
			for (int h = 0; h < Inst.Nh; ++h) {
				for (int v: Inst.Vehicles[h+Inst.Np]) {
					for (int j =0;j<Inst.node; j++) {
						if(Inst.dist[h+Inst.Np][j] > (Inst.WorkVehicle[v] / 2.0)){
							for (int k = 0; k < Inst.Nk; k++){
								for (int c = 0; c < Inst.Nc; c++){
									//cout<<w[j][t][v][k][c]<<endl;
									//masterModel.add( w[j][t][v][k][c] == 0);
									w[j][t][v][k][c].setUB(0);
									
								}
							}
						}
					}
				}
			}
		}
	}

	if(Inst.ImprovedCut>=1){
		if(Inst.ImprovedCut==1){
			for (int i = 0; i < Inst.node; ++i) {
				y[i] = IloArray<IloBoolVarArray>(env, Inst.Nt);
					for (int t = 0; t < Inst.Nt; ++t) {
						y[i][t] = IloBoolVarArray(env, Inst.Nv);
						for (int v = 0; v < Inst.Nv; ++v) {
							ostringstream varName;
							varName << "y_" << i <<"_"<<t <<"_"<<v;
							y[i][t][v] = IloBoolVar(env, varName.str().c_str());
						}	
					}
			}
		}else{
			for (int i = 0; i < Inst.node; ++i) {
				yr[i] = IloArray<IloNumVarArray>(env, Inst.Nt);
					for (int t = 0; t < Inst.Nt; ++t) {
						yr[i][t] = IloNumVarArray(env, Inst.Nv);
						for (int v = 0; v < Inst.Nv; ++v) {
							ostringstream varName;
							varName << "yr_" << i <<"_"<<t <<"_"<<v;
							yr[i][t][v] = IloNumVar(env, 0.0,1.0,varName.str().c_str());
						}	
					}
			}
		}
		for (int i = 0; i < Inst.node; ++i) {
			for (int t = 0; t < Inst.Nt; ++t) {
				for (int v = 0; v < Inst.Nv; ++v) {
					for (int k = 0; k < Inst.Nk; ++k) {
						for (int c = 0; c < Inst.Nc; ++c) {
							if(i<Inst.Np+Inst.Nh || i > Inst.Np + Inst.Nh + Inst.Nc -1 || c==i-Inst.Np-Inst.Nh ){
								if(Inst.ImprovedCut==1)
									masterModel.add(w[i][t][v][k][c] <= y[i][t][v]);
								else
									masterModel.add(w[i][t][v][k][c] <= yr[i][t][v]);
							}
						}
					}
				}
			}
		}
		
	}else{
		for (int i = 0; i < Inst.node; ++i) {
			for (int t = 0; t < Inst.Nt; ++t) {
				for (int v = 0; v < Inst.Nv; ++v) {
					for (int k = 0; k < Inst.Nk; ++k) {
						for (int c = 0; c < Inst.Nc; ++c) {
					
						
							masterModel.add(w[i][t][v][k][c] <=1);
						
					
						}
					}
				}
			}
		}
	}
}

void createWorkerModel(IloEnv& env, IloModel& workerModel, MyInstance Inst,IloArray<IloArray<IloNumVar>>& u,IloArray<IloArray<IloArray<IloBoolVar>>>& x) {

	
	for (int i = 0; i < Inst.node; ++i) {
		u[i] = IloArray<IloNumVar>(env, Inst.TourHub);
		for (int r = 0; r < Inst.TourHub; ++r) {
			std::ostringstream varName;
			varName << "u_" << i << "_" << r;
			u[i][r] = IloNumVar(env, 0, Inst.node, ILOINT, varName.str().c_str());
		}
	}

	// Define the binary variables x[i, j, r]
	for (int i = 0; i < Inst.node; ++i) {
		x[i] = IloArray<IloArray<IloBoolVar>>(env, Inst.node);
		for (int j = 0; j < Inst.node; ++j) {
			if(i!=j){
				x[i][j] = IloArray<IloBoolVar>(env, Inst.TourHub);
				for (int r = 0; r < Inst.TourHub; ++r) {
					std::ostringstream varName;
					varName << "x_" << i << "_" << j << "_" << r;
					x[i][j][r] = IloBoolVar(env, varName.str().c_str());
				}
			}
		}
	}
	IloExpr objective(env);
	for (int i = 0; i < Inst.node; ++i) {
		for (int j = 0; j < Inst.node; ++j) {
			for (int k = 0; k < Inst.TourHub; ++k) {
				if (j != i) {
					objective += Inst.dist[i][j] * x[i][j][k];
				}
			}
		}
	}
	
	
	workerModel.add(IloMinimize(env, objective));
	objective.end();
}

void GenWorkerModel(IloEnv& env, IloModel& workerModel, MyInstance Inst,IloArray<IloArray<IloNumVar>>& u,IloArray<IloArray<IloArray<IloBoolVar>>>& x, vector<int> NodeSub, vector<int> dem, int MaxTour, int MaxWork,int MaxCap) {
	IloConstraintArray cons(workerModel.getEnv());
	for (IloModel::Iterator it(workerModel); it.ok(); ++it) {
		if ( (*it).isConstraint() )
			cons.add((*it).asConstraint());
		}
		workerModel.remove(cons);
	cons.endElements();
	cons.end();
   IloExpr expr(env);
   int NbNode= (int) NodeSub.size();
   for (int j = 1; j < NbNode; ++j) {   
	  expr.clear();
	  //assert(dem[NodeSub[j]]>0);
	  if(dem[NodeSub[j]]>0){
		for (int i = 0; i < NbNode; ++i) {
			if (NodeSub[i] != NodeSub[j]) {
				for (int r = 0; r < MaxTour; ++r) {
					expr += x[NodeSub[i]][NodeSub[j]][r];
				}
			}
		}
	  
     	workerModel.add(expr == 1);
		}
   }
   for (int i = 0; i < NbNode; ++i) {
		if(i==0 || dem[NodeSub[i]]>0){
			for (int r = 0; r < MaxTour; ++r) {
				IloExpr flowdepot_in(env);
				expr.clear();
				for (int j = 0; j < NbNode; ++j) {
					if (NodeSub[i] != NodeSub[j] && (j==0 || dem[NodeSub[j]]>0)) {
						expr += x[NodeSub[i]][NodeSub[j]][r];
						flowdepot_in += x[NodeSub[j]][NodeSub[i]][r];
					}
				}
				workerModel.add(expr == flowdepot_in);
				flowdepot_in.end();
			}
		}
   }
   expr.clear();

	if(Inst.NoMaxWork==0){
		for (int i = 0; i < NbNode; ++i) {
			for (int j = 0; j < NbNode; ++j) {
				if(NodeSub[i]!=NodeSub[j]){
					for (int r = 0; r < MaxTour; ++r) {
						expr += x[NodeSub[i]][NodeSub[j]][r] * Inst.dist[NodeSub[i]][NodeSub[j]];
					}
				}
			}
		}
		workerModel.add(expr <= MaxWork);
	}
	expr.clear();
	for (int i = 1; i < NbNode; ++i) {
		for (int j = 1; j < NbNode; ++j) {
			if (NodeSub[i] != NodeSub[j]) {
				for (int r = 0; r < MaxTour; ++r) {
					workerModel.add(u[NodeSub[i]][r] - u[NodeSub[j]][r] + NbNode * x[NodeSub[i]][NodeSub[j]][r] + (NbNode - 2) * x[NodeSub[j]][NodeSub[i]][r] <= NbNode - 1);
				}
			}
		}
	}

	for (int r = 0; r < MaxTour; ++r) {
		expr.clear();
		for (int i = 0; i < NbNode; ++i) {
			for (int j = 0; j < NbNode; ++j) {
				if (NodeSub[i] != NodeSub[j]) {
					expr += x[NodeSub[i]][NodeSub[j]][r] * dem[NodeSub[j]];
				}
			}
		}
		workerModel.add(expr <= MaxCap);
	}
   expr.end();
}

/*void createWorkerModelFeas(IloEnv& env, IloModel& workerModel,vector<int> NodeSub, IloArray<IloArray<IloNumVar>>& u,IloArray<IloArray<IloArray<IloBoolVar>>>& x, int MaxTour) {
	
	cout<<"begin create "<<endl;
	cout<<NodeSub<<endl;
	int NbNode= (int) NodeSub.size();
	for (int i = 0; i < NbNode; ++i) {
		u[NodeSub[i]] = IloArray<IloNumVar>(env, MaxTour);
		for (int r = 0; r < MaxTour; ++r) {
			std::ostringstream varName;
			varName << "u_" << NodeSub[i] << "_" << r;
			u[NodeSub[i]][r] = IloNumVar(env, 0, NbNode, ILOINT, varName.str().c_str());
		}
	}
	cout<<"u  "<<endl;
	// Define the binary variables x[i, j, r]
	for (int i = 0; i <NbNode; ++i) {
		x[NodeSub[i]] = IloArray<IloArray<IloBoolVar>>(env, NbNode);
		for (int j = 0; j < NbNode; ++j) {
			cout<<"i "<<i<<" "<<j<<endl;
			if(i!=j){
				cout<<NodeSub[i]<<" "<<NodeSub[j]<<endl;
				x[NodeSub[i]][NodeSub[j]] = IloArray<IloBoolVar>(env, MaxTour);
				for (int r = 0; r < MaxTour; ++r) {
					std::ostringstream varName;
					varName << "x_" << NodeSub[i] << "_" << NodeSub[j] << "_" << r;
					cout<<r<<endl;
					x[NodeSub[i]][NodeSub[j]][r] = IloBoolVar(env, varName.str().c_str());
					cout<<x[NodeSub[i]][NodeSub[j]][r]<<endl;
				}
			}
		}
	}
	cout<<"enf created "<<endl;
}*/
void createWorkerModelFeas(IloEnv& env, IloModel& workerModel, MyInstance Inst,IloArray<IloArray<IloNumVar>>& u,IloArray<IloArray<IloArray<IloBoolVar>>>& x) {

	
	for (int i = 0; i < Inst.node; ++i) {
		u[i] = IloArray<IloNumVar>(env, Inst.TourHub);
		for (int r = 0; r < Inst.TourHub; ++r) {
			std::ostringstream varName;
			varName << "u_" << i << "_" << r;
			u[i][r] = IloNumVar(env, 0, Inst.node, ILOINT, varName.str().c_str());
		}
	}

	// Define the binary variables x[i, j, r]
	for (int i = 0; i < Inst.node; ++i) {
		x[i] = IloArray<IloArray<IloBoolVar>>(env, Inst.node);
		for (int j = 0; j < Inst.node; ++j) {
			if(i!=j){
				x[i][j] = IloArray<IloBoolVar>(env, Inst.TourHub);
				for (int r = 0; r < Inst.TourHub; ++r) {
					std::ostringstream varName;
					varName << "x_" << i << "_" << j << "_" << r;
					x[i][j][r] = IloBoolVar(env, varName.str().c_str());
				}
			}
		}
	}
}
void GenWorkerModelFeas(IloEnv& env, IloModel& workerModel, MyInstance Inst,IloArray<IloArray<IloNumVar>>& u,IloArray<IloArray<IloArray<IloBoolVar>>>& x, vector<int> NodeSub, vector<int> dem, int MaxTour, int MaxWork,int MaxCap) {
	IloConstraintArray cons(workerModel.getEnv());
	for (IloModel::Iterator it(workerModel); it.ok(); ++it) {
		if ( (*it).isConstraint() )
			cons.add((*it).asConstraint());
		}
		workerModel.remove(cons);
	cons.endElements();
	cons.end();
	int NbNode= (int) NodeSub.size();
	IloExpr expr(env);
	for (int j = 1; j < NbNode; ++j) {   
		expr.clear();
		if(dem[NodeSub[j]]>0){
			for (int i = 0; i < NbNode; ++i) {
				if (i != j) {
					for (int r = 0; r < MaxTour; ++r) {
						expr += x[NodeSub[i]][NodeSub[j]][r];
					}	
				}
			}
			workerModel.add(expr == 1);
		}
	}

	IloExpr flowdepot_in(env);
	for (int i = 0; i < NbNode; ++i) {
		if(i==0 || dem[NodeSub[i]]>0){
			for (int r = 0; r < MaxTour; ++r) {
				expr.clear();
				flowdepot_in.clear();
				for (int j = 0; j < NbNode; ++j) {
					if (i != j && (j==0 || dem[NodeSub[j]]>0)) {
						expr += x[NodeSub[i]][NodeSub[j]][r];
						flowdepot_in += x[NodeSub[i]][NodeSub[j]][r];
					}
				}
				workerModel.add(expr == flowdepot_in);
			}
		}
	}
	flowdepot_in.end();
	for (int i = 0; i < NbNode; ++i) {
		for (int j = 0; j < NbNode; ++j) {
			if(NodeSub[i]!=NodeSub[j]){
				for (int r = 0; r < MaxTour; ++r) {
					expr += x[NodeSub[i]][NodeSub[j]][r] * Inst.dist[NodeSub[i]][NodeSub[j]];
				}
			}
		}
	}
	workerModel.add(expr <= MaxWork);
	expr.clear();
	for (int i = 1; i < NbNode; ++i) {
		for (int j = 1; j < NbNode; ++j) {
			if (i != j) {
				for (int r = 0; r < MaxTour; ++r) {
					workerModel.add(u[NodeSub[i]][r] - u[NodeSub[j]][r] + NbNode * x[NodeSub[i]][NodeSub[j]][r] + (NbNode - 2) * x[NodeSub[j]][NodeSub[i]][r] <= NbNode - 1);
				}
			}
		}
	}
	
	for (int r = 0; r < MaxTour; ++r) {
		expr.clear();
		for (int i = 0; i < NbNode; ++i) {
			for (int j = 0; j < NbNode; ++j) {
				if (i!= j) {
					expr += x[NodeSub[i]][NodeSub[j]][r] * dem[NodeSub[j]];
				}
			}
		}
		workerModel.add(expr <= MaxCap);
	}
	
	expr.end();
}

void GenWorkerModelHub(IloEnv& env, IloModel& workerModel, MyInstance Inst,IloArray<IloArray<IloNumVar>>& u,IloArray<IloArray<IloArray<IloBoolVar>>>& x, vector<int> NodeSub, vector<int> dem, int MaxTour, int MaxWork,int MaxCap) {
	int NbNode= (int) NodeSub.size();
	for (int i = 0; i < NbNode; ++i) {
		u[i] = IloArray<IloNumVar>(env, MaxTour);
		for (int r = 0; r < MaxTour; ++r) {
			std::ostringstream varName;
			varName << "u_" << i << "_" << r;
			u[i][r] = IloNumVar(env, 0, NbNode, ILOINT, varName.str().c_str());
		}
	}

	// Define the binary variables x[i, j, r]
	for (int i = 0; i <NbNode; ++i) {
		x[i] = IloArray<IloArray<IloBoolVar>>(env, NbNode);
		for (int j = 0; j < NbNode; ++j) {
			if(i!=j){
				x[i][j] = IloArray<IloBoolVar>(env, MaxTour);
				for (int r = 0; r < MaxTour; ++r) {
					std::ostringstream varName;
					varName << "x_" << i << "_" << j << "_" << r;
					x[i][j][r] = IloBoolVar(env, varName.str().c_str());
				}
			}
		}
	}
	IloExpr objective(env);
	for (int i = 0; i < NbNode; ++i) {
		for (int j = 0; j < NbNode; ++j) {
			for (int k = 0; k < MaxTour; ++k) {
				if (j != i) {
					objective += Inst.dist[NodeSub[i]][NodeSub[j]] * x[i][j][k];
				}
			}
		}
	}
	workerModel.add(IloMinimize(env, objective));
	objective.end();
	
	IloExpr expr(env);
	for (int j = 1; j < NbNode; ++j) {   
		expr.clear();
		for (int i = 0; i < NbNode; ++i) {
			if (i != j) {
				for (int r = 0; r < MaxTour; ++r) {
					expr += x[i][j][r];
				}	
			}
		}
		workerModel.add(expr == 1);
	}
	for (int i = 0; i < NbNode; ++i) {
		for (int r = 0; r < MaxTour; ++r) {
			IloExpr flowdepot_in(env);
			expr.clear();
			for (int j = 0; j < NbNode; ++j) {
				if (i != j) {
					expr += x[i][j][r];
					flowdepot_in += x[j][i][r];
				}
			}
			workerModel.add(expr == flowdepot_in);
			flowdepot_in.end();
		}
	}
	

	for (int r = 0; r < MaxTour; ++r) {
		expr.clear();
		for (int i = 0; i < NbNode; ++i) {
			for (int j = 0; j < NbNode; ++j) {
				if(i!=j){
					expr += x[i][j][r] * Inst.dist[NodeSub[i]][NodeSub[j]];
				}
			}
		}
		workerModel.add(expr <= MaxWork);
	}
	
	/*for (int r = 0; r < MaxTour; ++r) {
		for (int i = 0; i < NbNode; ++i) {
			for (int j = i+1; j < NbNode; ++j) {
				if(NodeSub[i]==NodeSub[j])
					workerModel.add(x[i][j][r] +x[j][i][r] <=1);
			}
		}
	}*/
	
	expr.clear();
	for (int i = 1; i < NbNode; ++i) {
		for (int j = 1; j < NbNode; ++j) {
			if (i != j) {
				for (int r = 0; r < MaxTour; ++r) {
					workerModel.add(u[i][r] - u[j][r] + NbNode * x[i][j][r] + (NbNode - 2) * x[j][i][r] <= NbNode - 1);
				}
			}
		}
	}

	
	for (int r = 0; r < MaxTour; ++r) {
		expr.clear();
		for (int i = 0; i < NbNode; ++i) {
			for (int j = 0; j < NbNode; ++j) {
				if (i!= j) {
					expr += x[i][j][r] * dem[j];
				}
			}
		}
		workerModel.add(expr <= MaxCap);
	}
	
	expr.end();
}
vector<vector<int>> Binpacking(vector<int> Pick,vector<int> Deli, int MaxWork,int nbvehicle, vector<pair<int,int>> conflictPick, vector<pair<int,int>> conflictDeli) {
	IloEnv env;
	IloModel model(env);
	int num_items=(int) Pick.size() + Deli.size();
	int num_bins=nbvehicle;
	
	vector<vector<int>> UsedBin(num_bins);
	// Decision variables x_ij
	IloArray<IloBoolVarArray> xpick(env, num_items);
	for (size_t i = 0; i < Pick.size(); i++) {
		xpick[i] = IloBoolVarArray(env, num_bins);
		for (int j = 0; j < num_bins; ++j) {
			std::ostringstream varName;
			varName << "xp_" << i << "_" << j;
			xpick[i][j] = IloBoolVar(env, varName.str().c_str());
		}
		
	}
	IloArray<IloBoolVarArray> xdeli(env, num_items);
	for (size_t i = 0; i < Deli.size(); i++) {
		xdeli[i] = IloBoolVarArray(env, num_bins);
		for (int j = 0; j < num_bins; ++j) {
			std::ostringstream varName;
			varName << "xd_" << i << "_" << j;
			xdeli[i][j] = IloBoolVar(env, varName.str().c_str());
		}
	}
	// Objective: Minimize the number of bins used
	IloBoolVarArray y(env, num_bins); // y[j] = 1 if bin j is used
	for (int j = 0; j < num_bins; j++) {
		y[j] = IloBoolVar(env);
	}
	//model.add(IloMinimize(env, IloSum(y)));

	// Constraint 1: Each item must be placed in exactly one bin
	for (size_t i = 0; i <  Pick.size(); i++) {
		IloExpr sum_x(env);
		for (int j = 0; j < num_bins; j++) {
			sum_x += xpick[i][j];
		}
		model.add(sum_x == 1); // Each item assigned to one bin
		sum_x.end();
	}
	for (size_t i = 0; i < Deli.size(); i++) {
		IloExpr sum_x(env);
		for (int j = 0; j < num_bins; j++) {
			sum_x += xdeli[i][j];
		}
		model.add(sum_x == 1); // Each item assigned to one bin
		sum_x.end();
	}
	// Constraint 2: Bin capacity
	for (int j = 0; j < num_bins; j++) {
		IloExpr bin_weight(env);
		for (size_t i = 0; i < Pick.size(); i++) {
			bin_weight += Pick[i] * xpick[i][j];
		}
		for (size_t i = 0; i < Deli.size(); i++) {
			bin_weight += Deli[i] * xdeli[i][j];
		}
		model.add(bin_weight <= MaxWork * y[j]); // Link x_ij to y_j
		bin_weight.end();
	}
	
	for (size_t i = 0; i < conflictPick.size(); i++){
		IloExpr conf(env);
		for (int j = 0; j < num_bins; j++) {
			model.add(xpick[conflictPick[i].first][j] + xpick[conflictPick[i].second][j] <= 1);
		}
	}

	for (size_t i = 0; i < conflictDeli.size(); i++){
		IloExpr conf(env);
		for (int j = 0; j < num_bins; j++) {
			model.add(xdeli[conflictDeli[i].first][j] + xdeli[conflictDeli[i].second][j] <= 1);
		}
	}
	
	// Solve the model
	IloCplex cplex(model);
	cplex.setOut(env.getNullStream()); // Disable output for cleaner display
	if (cplex.solve()) {
		//std::cout << "Optimal solution found!" << std::endl;
		//std::cout << "Bins used: " << cplex.getObjValue() << std::endl;
		for (int j = 0; j < num_bins; j++) {
			if (cplex.getValue(y[j]) > 0.5) { // Bin j is used
				//std::cout << "Bin " << j  << ": ";
				for (size_t i = 0; i < Pick.size(); i++) {
					if (cplex.getValue(xpick[i][j]) > 0.5) { // Item i in bin j
						//std::cout << "PickItem " << i  << " ";
						UsedBin[j].push_back(i+1);
					}
				}
				for (size_t i = 0; i < Deli.size(); i++) {
					if (cplex.getValue(xdeli[i][j]) > 0.5) { // Item i in bin j
						//std::cout << "DeliItem " << i << " ";
						UsedBin[j].push_back(-i-1);
					}
				}
				//std::cout << std::endl;
			}
		}
	} else {
		std::cout << "No feasible solution found!" << std::endl;
		/*cout<<"Pick "<<Pick<<endl;
		cout<<" Delu "<<Deli<<endl;
		cout<<MaxWork<<" "<<nbvehicle<<endl;*/
		//cplex.exportModel("binpack.lp");
		UsedBin.clear();
	}

	// End environment
	env.end();
	return UsedBin;
}


void AddFeasCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w, vector<array<int,3>> Var,vector<array<int,3>> Var2, IloConstraintArray& Addcuts,int currt){
	IloExpr expr(env);
	if(Inst.LessCut==2 ){
		for (int v = 0; v < Inst.Nv; v++){
			if(Inst.StartVehicle[v]==currnode){
				expr.clear();
				for (int i = 0; i < (int)Var.size(); ++i) {
					expr += w[Var[i][0]][currt][v][Var[i][1]][Var[i][2]];  // Build an expression with the variables in OneVal
				}
				for (int i = 0; i < (int)Var2.size(); ++i) {
					expr += w[Var2[i][0]][currt][v][Var2[i][1]][Var2[i][2]];  // Build an expression with the variables in OneVal
				}
				Inst.NbFeasCut++;
				Addcuts.add(expr <=  (int)(Var2.size()+Var.size()-1));  // Add a constraint using OneVal
			}
		}
	}else{
		for (int t = 0; t < Inst.Nt; t++){
			if(currnode >= Inst.Np || Inst.Prod_av[currnode][t]){
				for (int v = 0; v < Inst.Nv; v++){
					if(Inst.StartVehicle[v]==currnode){
						expr.clear();
						for (int i = 0; i < (int)Var.size(); ++i) {
							expr += w[Var[i][0]][t][v][Var[i][1]][Var[i][2]];  // Build an expression with the variables in OneVal
						}
						for (int i = 0; i < (int)Var2.size(); ++i) {
							expr += w[Var2[i][0]][t][v][Var2[i][1]][Var2[i][2]];  // Build an expression with the variables in OneVal
						}
						Inst.NbFeasCut++;
						Addcuts.add(expr <=  (int)(Var2.size()+Var.size()-1));  // Add a constraint using OneVal
					}
				}
			}
		}
	}
	expr.end();
}

void ImprovedAddFeasCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloBoolVarArray>>& y, vector<int> Var,vector<int> Var2, IloConstraintArray& Addcuts,int currt, IloArray<IloArray<IloNumVarArray>> yr){
	IloExpr expr(env);
	if(Inst.LessCut==2){
		for (int v = 0; v < Inst.Nv; v++){
			if(Inst.StartVehicle[v]==currnode){
				expr.clear();
				for (int i = 0; i < (int)Var.size(); ++i) {
					if(Var[i]!=currnode){
						if(Inst.ImprovedCut==1)
							expr += y[Var[i]][currt][v];
						else
							expr += yr[Var[i]][currt][v];
					}
				}
				for (int i = 0; i < (int)Var2.size(); ++i) {
					if(Var2[i]!=currnode){
						if(Inst.ImprovedCut==1)
							expr += y[Var2[i]][currt][v];  
						else
							expr += yr[Var2[i]][currt][v];  
					}
				}
				Inst.NbFeasCut++;
				Addcuts.add(expr <=  (int)(Var2.size()+Var.size()-1));  
			}
		}
	}else{
		for (int t = 0; t < Inst.Nt; t++){
			if(currnode >= Inst.Np || Inst.Prod_av[currnode][t]){
				for (int v = 0; v < Inst.Nv; v++){
					if(Inst.StartVehicle[v]==currnode){
						expr.clear();
						for (int i = 0; i < (int)Var.size(); ++i) {
							if(Var[i]!=currnode){
								if(Inst.ImprovedCut==1)
									expr += y[Var[i]][t][v];
								else 
									expr += yr[Var[i]][t][v];
							}
						}
						for (int i = 0; i < (int)Var2.size(); ++i) {
							if(Var2[i]!=currnode){
								if(Inst.ImprovedCut==1)
									expr += y[Var2[i]][t][v];
								else
									expr += yr[Var2[i]][t][v];
							}
						}
						Inst.NbFeasCut++;
						
						int nbelm=0;
						if(!Var.empty())
							nbelm+=(int) Var.size()-1;
						if(!Var2.empty())
							nbelm+=(int) Var2.size()-1;

						Addcuts.add(expr <= nbelm-1);  
					}
				}
			}
		}
	}
	expr.end();
}

vector<bool> StrengthenFeasCut(vector<int> NodeSub, MyInstance& Inst, vector<int> dem, int tour, int MaxWork,int MaxCap, bool imp, vector<array<int,3>> Commands) {
	IloEnv env2;	
	IloArray<IloArray<IloNumVar>> u2(env2, Inst.node);
	IloArray<IloArray<IloArray<IloBoolVar>>> x2(env2, Inst.node);
	IloModel WorkerModel2(env2);
	//cout<<ProblemHubDeli[h]<<endl;
	//cout<<DemandsProblemHubDeli[h]<<endl;
	createWorkerModelFeas(env2, WorkerModel2,Inst,u2,x2);
	IloCplex WorkerCplex2(WorkerModel2);
	WorkerCplex2.setParam(IloCplex::Param::Threads, 1);
	WorkerCplex2.setParam(IloCplex::Param::MIP::Display, 0);
	WorkerCplex2.setParam(IloCplex::Param::TimeLimit, 5);
	WorkerCplex2.setOut(env2.getNullStream());  // Suppress log output
	WorkerCplex2.setWarning(env2.getNullStream());
	WorkerCplex2.setParam(IloCplex::Param::Emphasis::MIP, IloCplex::MIPEmphasisFeasibility);
	WorkerCplex2.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);
	int currsize,start;
	if(imp){
		currsize=NodeSub.size();
		start=1;
	}
	else{
		currsize=Commands.size();
		start=0;
	}

	vector<bool> MinExpl(currsize,true);
	int randnumber=0,OldDem=0;
	//WorkerCplex.setParam(IloCplex::Param::MIP::Limits::Solutions, 1);
	if(Inst.StrenghtenFeasCut==2)
		randnumber =rand()%101;
	if(randnumber<50){
		for (int i = start; i < currsize; i++){
			if(!imp)
				dem[Commands[i][0]]-=Inst.demands[Commands[i][2]][Commands[i][1]]*Inst.Psize[Commands[i][1]];
			else{
				OldDem=dem[NodeSub[i]];
				dem[NodeSub[i]]=0;
			}
			GenWorkerModel(env2, WorkerModel2,Inst,u2,x2,NodeSub,dem, tour,MaxWork,MaxCap);
			WorkerCplex2.solve();
			if(WorkerCplex2.getStatus()==IloAlgorithm::Infeasible){
				MinExpl[i]=false;
			}else{
				if(!imp)
					dem[Commands[i][0]]+=Inst.demands[Commands[i][2]][Commands[i][1]]*Inst.Psize[Commands[i][1]];
				else
					dem[NodeSub[i]]=OldDem;
			}
		}
	}else{
		for (int i =  currsize; i > 0; i--){
			if(!imp)
				dem[Commands[i][0]]-=Inst.demands[Commands[i][2]][Commands[i][1]]*Inst.Psize[Commands[i][1]];
			else{
				OldDem=dem[NodeSub[i]];
				dem[NodeSub[i]]=0;
			}
			GenWorkerModel(env2, WorkerModel2,Inst,u2,x2,NodeSub,dem, tour,MaxWork,MaxCap);
			WorkerCplex2.solve();
			if(WorkerCplex2.getStatus()==IloAlgorithm::Infeasible){
				MinExpl[i]=false;
			}else{
				if(!imp)
					dem[Commands[i][0]]+=Inst.demands[Commands[i][2]][Commands[i][1]]*Inst.Psize[Commands[i][1]];
				else
					dem[NodeSub[i]]=OldDem;
			}
		}
	}
	WorkerCplex2.end();	
	WorkerModel2.end();
	env2.end();
	return MinExpl;
}
void AddOptCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w,  IloArray<IloArray<IloNumVar>>& sigma, float Opt, vector<array<int,3>> Var, vector<array<int,3>> Var2, IloConstraintArray& Addcuts, int currt){
	IloExpr expr(env);
	float init;
	/*cout<<"OPT is "<<Opt<<endl;
	for (int i = 0; i < (int) Var.size(); i++)
	{
		cout<<Var[i][0]<<" "<<Var[i][1]<<" "<<Var[i][2]<<" "<<endl;
	}
	cout<<"VARDELI"<<endl;
	for (int i = 0; i < (int) Var2.size(); i++)
	{
		cout<<Var2[i][0]<<" "<<Var2[i][1]<<" "<<Var2[i][2]<<" "<<endl;
	}
	cout<<endl;
	cout<<" ADD OPT CUT"<<endl;*/
	if(Inst.LessCut==2 ){
		for (int v = 0; v < Inst.Nv; v++){
			if(Inst.StartVehicle[v]==currnode){
				expr.clear();
				init=Opt;
				for (int i = 0; i < (int)Var.size(); ++i) {
					init-=2*Inst.dist[currnode][Var[i][0]];
					expr +=  w[Var[i][0]][currt][v][Var[i][1]][Var[i][2]] * Inst.dist[currnode][Var[i][0]];  // Build an expression with the variables in OneVal
				}
				for (int i = 0; i < (int)Var2.size(); ++i) {
					expr += w[Var2[i][0]][currt][v][Var2[i][1]][Var2[i][2]] *Inst.dist[currnode][Var2[i][0]]; // Build an expression with the variables in OneVal
					init-=2*Inst.dist[currnode][Var2[i][0]];
				}
				Inst.NbOptCut++;
				Addcuts.add(init + 2*expr <= sigma[currt][v]);  // Add a constraint using OneVal
				
			}
		}
	}
	else{
		for (int t = 0; t < Inst.Nt; t++){
			//Add a cut only if the current node is a hub or an available producer.
			if(currnode >= Inst.Np || Inst.Prod_av[currnode][t]){
				for (int v = 0; v < Inst.Nv; v++){
					if(Inst.StartVehicle[v]==currnode){
						expr.clear();
						init=Opt;
						for (int i = 0; i < (int)Var.size(); ++i) {
							init-=2*Inst.dist[currnode][Var[i][0]];
							expr +=  w[Var[i][0]][t][v][Var[i][1]][Var[i][2]] * Inst.dist[currnode][Var[i][0]];  // Build an expression with the variables in OneVal
						}
						for (int i = 0; i < (int)Var2.size(); ++i) {
							expr += w[Var2[i][0]][t][v][Var2[i][1]][Var2[i][2]] *Inst.dist[currnode][Var2[i][0]]; // Build an expression with the variables in OneVal
							init-=2*Inst.dist[currnode][Var2[i][0]];
						}
						Inst.NbOptCut++;
						Addcuts.add(init + 2*expr <= sigma[t][v]);  // Add a constraint using OneVal
						
					}
				}
			}
		}
	}
	expr.end();
}


void AddImprovedProdOptCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloBoolVarArray>> y,  IloArray<IloArray<IloNumVar>> sigma, float Opt, vector<int> Var,vector<int> Var2, IloConstraintArray& Addcuts, int currt, IloArray<IloArray<IloNumVarArray>> yr){
	IloExpr expr(env);
	float init;
	if(Inst.LessCut==2){
		for (int v = 0; v < Inst.Nv; v++){
			if(Inst.StartVehicle[v]==currnode){
				expr.clear();
				init=Opt;
				for (int i = 0; i < (int)Var.size(); ++i) {
					init-=2*Inst.dist[currnode][Var[i]];
					if(Inst.ImprovedCut==1)
						expr +=  y[Var[i]][currt][v] * Inst.dist[currnode][Var[i]];  // Build an expression with the variables in OneVal
					else
						expr +=  yr[Var[i]][currt][v] * Inst.dist[currnode][Var[i]]; 
				}
				for (int i = 0; i < (int)Var2.size(); ++i) {
					if(Inst.ImprovedCut==1)
						expr += y[Var2[i]][currt][v] *Inst.dist[currnode][Var2[i]]; // Build an expression with the variables in OneVal
					else
						expr += yr[Var2[i]][currt][v] *Inst.dist[currnode][Var2[i]];
					init-=2*Inst.dist[currnode][Var2[i]];
				}
				Inst.NbOptCut++;
				Addcuts.add(init + 2*expr <= sigma[currt][v]);  // Add a constraint using OneVal
			}
		}
	}
	else{
		for (int t = 0; t < Inst.Nt; t++){
			//Add a cut only if the current node is a hub or an available producer.
			if(currnode >= Inst.Np || Inst.Prod_av[currnode][t]){
				for (int v = 0; v < Inst.Nv; v++){
					if(Inst.StartVehicle[v]==currnode){
						expr.clear();
						init=Opt;
						for (int i = 0; i < (int)Var.size(); ++i) {
							init-=2*Inst.dist[currnode][Var[i]];
							if(Inst.ImprovedCut==1)
								expr +=  y[Var[i]][t][v] * Inst.dist[currnode][Var[i]];  // Build an expression with the variables in OneVal
							else
								expr +=  yr[Var[i]][t][v] * Inst.dist[currnode][Var[i]];
							
						}
						for (int i = 0; i < (int)Var2.size(); ++i) {
							if(Inst.ImprovedCut==1)
								expr += y[Var2[i]][t][v] *Inst.dist[currnode][Var2[i]]; // Build an expression with the variables in OneVal
							else 
								expr += yr[Var2[i]][t][v] *Inst.dist[currnode][Var2[i]];
							init-=2*Inst.dist[currnode][Var2[i]];
						}
						Inst.NbOptCut++;
						Addcuts.add(init + 2*expr <= sigma[t][v]);  // Add a constraint using OneVal
						assert(init<=0);
					}
				}
			}
		}
	}
	expr.end();
}
void AddMoreFeasCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w, vector<array<int,3>> VarPick,vector<array<int,3>> VarDeli, IloConstraintArray& Addcuts){
	vector<array<int,3>> tempvec=VarPick;
	bool InVarVec;
	int a;
	
	
	for (int i = 0; i < (int)VarPick.size(); i++){
		for (int k = 0; k < Inst.Nk; k++){
			for (int c = 0; c < Inst.Nc; c++){
				if(Inst.demands[c][k]*Inst.Psize[k] >= Inst.demands[VarPick[i][2]][VarPick[i][1]]*Inst.Psize[VarPick[i][1]] && VarPick[i][0]<Inst.Np && Inst.stocks[VarPick[i][0]][k]>=Inst.demands[c][k]){
					a=0;
					InVarVec=false;
					while(a < (int) VarPick.size() && !InVarVec){
						if(VarPick[a][1]==k && VarPick[a][2]==c)
							InVarVec=true;
						a++;
					}
					if(!InVarVec){
						tempvec[i]={VarPick[i][0],k,c};
						AddFeasCut(env,Inst,currnode,w,tempvec,VarDeli,Addcuts,0);
						tempvec[i]={VarPick[i][0],VarPick[i][1],VarPick[i][2]};
					}
				}
			}
		}
	}
	if(VarPick.size()>0)
		assert(currnode>=Inst.Np);
	tempvec=VarDeli;
	for (int i = 0; i < (int)VarDeli.size(); i++){
		for (int k = 0; k < Inst.Nk; k++){
			if(VarDeli[i][1]!=k && Inst.demands[VarDeli[i][2]][k]*Inst.Psize[k] >= Inst.demands[VarDeli[i][2]][VarDeli[i][1]]*Inst.Psize[VarDeli[i][1]]){
				a=0;
				InVarVec=false;
				while(a < (int) VarDeli.size() && !InVarVec){
					if(VarDeli[a][1]==k && VarDeli[a][2]==VarDeli[i][2])
						InVarVec=true;
					a++;
				}
				if(!InVarVec){
					tempvec[i]={VarDeli[i][0],k,VarDeli[i][2]};
					AddFeasCut(env,Inst,currnode,w,VarPick,tempvec,Addcuts,0);
					tempvec[i]={VarDeli[i][0],VarDeli[i][1],VarDeli[i][2]};
				}
			}
		}
	}
}

void AddMoreOptCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w,  IloArray<IloArray<IloNumVar>>& sigma, float Opt, vector<array<int,3>> VarPick, vector<array<int,3>> VarDeli, IloConstraintArray& Addcuts){
	vector<array<int,3>> tempvec=VarPick;
	bool InVarVec;
	int a;
	for (int i = 0; i < (int)VarPick.size(); i++){
		assert(currnode>=Inst.Np);
		for (int k = 0; k < Inst.Nk; k++){
			for (int c = 0; c < Inst.Nc; c++){
				if(Inst.demands[c][k]*Inst.Psize[k] >= Inst.demands[VarPick[i][2]][VarPick[i][1]]*Inst.Psize[VarPick[i][1]] && VarPick[i][0]<Inst.Np && Inst.stocks[VarPick[i][0]][k]>=Inst.demands[c][k]){
					a=0;
					InVarVec=false;
					while(a < (int) VarPick.size() && !InVarVec){
						if(VarPick[a][1]==k && VarPick[a][2]==c)
							InVarVec=true;
						a++;
					}
					if(!InVarVec){
						tempvec[i]={VarPick[i][0],k,c};
						AddOptCut(env,Inst,currnode,w,sigma,Opt,tempvec,VarDeli,Addcuts,0);
						tempvec[i]={VarPick[i][0],VarPick[i][1],VarPick[i][2]};
					}
				}
			}
		}
	}
	tempvec=VarDeli;
	for (int i = 0; i < (int)VarDeli.size(); i++){
		for (int k = 0; k < Inst.Nk; k++){
			if(VarDeli[i][1]!=k && Inst.demands[VarDeli[i][2]][k]*Inst.Psize[k] >= Inst.demands[VarDeli[i][2]][VarDeli[i][1]]*Inst.Psize[VarDeli[i][1]]){
				a=0;
				InVarVec=false;
				while(a < (int) VarDeli.size() && !InVarVec){
					if(VarDeli[a][1]==k && VarDeli[a][2]==VarDeli[i][2])
						InVarVec=true;
					a++;
				}
				if(!InVarVec){
					tempvec[i]={VarDeli[i][0],k,VarDeli[i][2]};
					AddOptCut(env,Inst,currnode,w,sigma,Opt,VarPick,tempvec,Addcuts,0);
					tempvec[i]={VarDeli[i][0],VarDeli[i][1],VarDeli[i][2]};
				}
			}
		}
	}
}
bool AddCutsFromHub(IloEnv& env, MyInstance& Inst, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w,  IloArray<IloArray<IloNumVar>> sigma, IloArray<IloArray<IloBoolVarArray>> y, IloArray<IloArray<IloNumVarArray>> yr, IloConstraintArray& AddCuts, int opt1, int opt2, vector<int> TourCostPick, vector<int> TourCostDeli, vector<vector<int>> TourPick, vector<vector<int>> TourDeli, vector<int> ProblemHubPick ,vector<int>  ProblemHubDeli, vector<int> DemandsProblemHubDeli, vector<int> DemandsProblemHubPick, vector<array<int,3>> VarPickHub, vector<array<int,3>> VarDeliHub, int currt, int currh){
	vector<array<int,3>> VarDeli;
	vector<int> DeliNode,PickNode;
	vector<array<int,3>> VarPick;
	int Costbin=0;
	if(Inst.PartialCut==2){
		for (size_t i = 0; i < TourDeli.size(); i++){
			if(!TourDeli[i].empty()){
				VarDeli.clear();
				DeliNode.clear();
				int newdem=0;
				DeliNode.push_back(currh+Inst.Np);

				for (size_t j = 0; j < TourDeli[i].size(); j++)
				{
					if(TourDeli[i][j]!=0){
						VarDeli.push_back(VarDeliHub[TourDeli[i][j]-1]);
						newdem+=DemandsProblemHubDeli[TourDeli[i][j]];
						DeliNode.push_back(ProblemHubDeli[TourDeli[i][j]]);
					}
				}
				if(Inst.ImprovedCut==0 || newdem > Inst.CapaHub){
					AddOptCut(env,Inst,currh+Inst.Np,w,sigma,TourCostDeli[i],VarDeli,{},AddCuts,currt);
				}else{
					AddImprovedProdOptCut(env,Inst,currh+Inst.Np,y,sigma,TourCostDeli[i],DeliNode,{},AddCuts,currt,yr);
				}
			}
		}
		for (size_t i = 0; i < TourPick.size(); i++){
			if(!TourPick[i].empty()){
				VarPick.clear();
				PickNode.clear();
				int newdem=0;
				PickNode.push_back(currh+Inst.Np);
				for (size_t j = 0; j < TourPick[i].size(); j++)
				{
					if(TourPick[i][j]!=0){
						VarPick.push_back(VarPickHub[TourPick[i][j]-1]);
						newdem+=DemandsProblemHubPick[TourPick[i][j]];
						PickNode.push_back(ProblemHubPick[TourPick[i][j]]);
					}
				}
				if(Inst.ImprovedCut==0 || newdem > Inst.CapaHub){
					AddOptCut(env,Inst,currh+Inst.Np,w,sigma,TourCostPick[i],VarPick,{},AddCuts,currt);
				}else{
					AddImprovedProdOptCut(env,Inst,currh+Inst.Np,y,sigma,TourCostPick[i],PickNode,{},AddCuts,currt,yr);
				}
			}
		}
	}
	if((int)TourPick.size()+(int)TourDeli.size()> Inst.Nvh || Inst.PartialCut!=2){
		vector<pair<int,int>> ConfPick, ConfDeli;
		for (size_t i = 0; i < TourPick.size(); i++){
			for (size_t j = i+1; j < TourPick.size(); j++){
				for (size_t k = 0; k < TourPick[i].size(); k++){
					for (size_t k2 = 0; k2 < TourPick[j].size(); k2++){
						//if(ProblemHubPick[TourPick[i][k]]==ProblemHubPick[TourPick[j][k2]])
							ConfPick.push_back({i,j});
					}
				}
				
			}
		}
		for (size_t i = 0; i < TourDeli.size(); i++){
			for (size_t j = i+1; j < TourDeli.size(); j++){
				for (size_t k = 0; k < TourDeli[i].size(); k++){
					for (size_t k2 = 0; k2 < TourDeli[j].size(); k2++){
						//if(ProblemHubDeli[TourDeli[i][k]]==ProblemHubDeli[TourDeli[j][k2]])
							ConfDeli.push_back({i,j});
					}
				}
				
			}
		}
		vector<vector<int>> Bins= Binpacking(TourCostPick,TourCostDeli,Inst.WorkHub,Inst.Nvh,ConfPick,ConfDeli);
		if(Bins.empty())
			return false;
		for (int i = 0; i < Inst.Nvh; i++){
			if(!Bins[i].empty() && (Inst.PartialCut!=2 ||Bins[i].size()>1)){
				VarDeli.clear();
				VarPick.clear();
				DeliNode.clear();
				PickNode.clear();
				PickNode.push_back(currh+Inst.Np);
				DeliNode.push_back(currh+Inst.Np);
				Costbin=0;
				for (size_t j = 0; j < Bins[i].size();j++){
					if(Bins[i][j]>0){
						Costbin+=TourCostPick[Bins[i][j]-1];
						for (size_t k = 0; k < TourPick[Bins[i][j]-1].size(); k++)
						{
							if(TourPick[Bins[i][j]-1][k]!=0){
								VarPick.push_back(VarPickHub[TourPick[Bins[i][j]-1][k]-1]);
								if(find(PickNode.begin(),PickNode.end(),ProblemHubPick[TourPick[Bins[i][j]-1][k]])==PickNode.end())
									PickNode.push_back(ProblemHubPick[TourPick[Bins[i][j]-1][k]]);
							}
						}
					}else{
						Costbin+=TourCostDeli[-Bins[i][j]-1];
						for (size_t k = 0; k < TourDeli[-Bins[i][j]-1].size(); k++)
						{
							if(TourDeli[-Bins[i][j]-1][k]!=0){
								VarDeli.push_back(VarDeliHub[TourDeli[-Bins[i][j]-1][k]-1]);
								if(find(DeliNode.begin(),DeliNode.end(),ProblemHubDeli[TourDeli[-Bins[i][j]-1][k]])==DeliNode.end())
									DeliNode.push_back(ProblemHubDeli[TourDeli[-Bins[i][j]-1][k]]);
							}
						}
					}
				}
				assert(Costbin<= Inst.WorkHub);
				if(PickNode.size()==1)
					PickNode.clear();
				if(DeliNode.size()==1)
					DeliNode.clear();
				if(Inst.ImprovedCut==0){
					AddOptCut(env,Inst,currh+Inst.Np,w,sigma,Costbin,VarPick,VarDeli,AddCuts,currt);
				}else{
					AddImprovedProdOptCut(env,Inst,currh+Inst.Np,y,sigma,Costbin,PickNode,DeliNode,AddCuts,currt,yr);
				}
			}
		}
	}
	return true;

}
int FindCuts(IloEnv& env, IloCplex& MasterCplex, IloCplex& WorkerCplex, IloModel& WorkerModel,  MyInstance& Inst, int sol, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w, IloArray<IloArray<IloBoolVarArray>> y, IloArray<IloArray<IloNumVarArray>> yr, IloArray<IloArray<IloNumVar>> sigma, IloArray<IloArray<IloNumVar>>& u,IloArray<IloArray<IloArray<IloBoolVar>>>& x, IloConstraintArray& AddCuts){
	pair<vector<int>,vector<int>> PickAndDel,DemandPickAndDel(vector<int>(Inst.node,0),vector<int>(Inst.node,0));
	vector<array<int,3>> VarPick, VarDeli;
	pair<int,int> TotalDem;
	vector<int> NodeSub,dem;
	float epsi = 1e-5,value;
	int upper=0;
	bool AlreadyAdded;
	vector<vector<int>> ProblemHubPick(Inst.Nh),ProblemHubDeli(Inst.Nh),DemandsProblemHubDeli(Inst.Nh),DemandsProblemHubPick(Inst.Nh);
	vector<vector<array<int,3>>> VarPickHub(Inst.Nh),VarDeliHub(Inst.Nh);
	vector<int> resHub(Inst.Nh,0);
	if(Inst.Output!=""){
		std::ofstream outFile(Inst.Output);
		outFile.close();
	}
	for (int t = 0; t < Inst.Nt; ++t) {
		for (int v = 0; v < Inst.Nv; v++){
			if(Inst.StartVehicle[v] >= Inst.Np || Inst.Prod_av[Inst.StartVehicle[v]][t]){
				PickAndDel.first.clear();
				PickAndDel.second.clear();
				PickAndDel.first.push_back(Inst.StartVehicle[v]);
				PickAndDel.second.push_back(Inst.StartVehicle[v]);
				fill(DemandPickAndDel.first.begin(),DemandPickAndDel.first.end(),0);
				fill(DemandPickAndDel.second.begin(),DemandPickAndDel.second.end(),0);
				VarDeli.clear();
				VarPick.clear();
				int tot=0;
				TotalDem.first=0;
				TotalDem.second=0;
				for (int i = 0; i < Inst.node; ++i) {
					if(i!=Inst.StartVehicle[v] && (Inst.StartVehicle[v]<Inst.Np || Inst.PairHub[Inst.StartVehicle[v]-Inst.Np].second !=i )){
						AlreadyAdded=false;
						for (int k = 0; k < Inst.Nk; ++k) {
							for (int c = 0; c < Inst.Nc; ++c) {
								if(Inst.demands[c][k]>0 && w[i][t][v][k][c].getUB()!=0){
									if(i<Inst.Np+Inst.Nh || i > Inst.Np + Inst.Nh + Inst.Nc -1 || c==i-Inst.Np-Inst.Nh ){
										if(sol==-1)
											value = MasterCplex.getValue(w[i][t][v][k][c]);
										else
											value = MasterCplex.getValue(w[i][t][v][k][c],sol);
										if(value>1-epsi){
											if(i<Inst.Np + Inst.Nh){
												//cout<<"pick "<<w[i][t][v][k][c]<<endl;
												if(!AlreadyAdded){
													AlreadyAdded=true;
													PickAndDel.first.push_back(i);
												}
												DemandPickAndDel.first[i]+=Inst.demands[c][k]*Inst.Psize[k];
												//cout<<Inst.demands[c][k]*Inst.Psize[k]<<" "<<endl;
												VarPick.push_back({i,k,c});
												TotalDem.first+=Inst.demands[c][k]*Inst.Psize[k];
											}else{
												//cout<<"depot "<<w[i][t][v][k][c]<<endl;
												if(!AlreadyAdded){
													AlreadyAdded=true;
													PickAndDel.second.push_back(i);
												}
												DemandPickAndDel.second[i]+=Inst.demands[c][k]*Inst.Psize[k];
												VarDeli.push_back({i,k,c});
												TotalDem.second+=Inst.demands[c][k]*Inst.Psize[k];
											}
										}
									}
								}
							}
							
						}
					}
				}
				vector<float> res;
				res.push_back(0);
				res.push_back(0);
				vector<float> resV2;
				resV2.push_back(-2);
				resV2.push_back(-2);
				bool CapUnf;
				for (int s = 0; s < 2; s++){
					CapUnf=false;
					if(s==1){
						NodeSub=PickAndDel.first;
						dem=DemandPickAndDel.first;
						if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
							for (size_t i = 0; i < VarPick.size(); i++){
								VarPickHub[Inst.StartVehicle[v]-Inst.Np].push_back(VarPick[i]);
							}
						}
					}
					else{
						NodeSub=PickAndDel.second;
						dem=DemandPickAndDel.second;
						if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1){
							for (size_t i = 0; i < VarDeli.size(); i++){
								VarDeliHub[Inst.StartVehicle[v]-Inst.Np].push_back(VarDeli[i]);
							}
						}
					}
					/*cout<<v <<" "<<t<<endl;
					cout<<NodeSub<<endl;*/
					vector<array<int,3>> VarPickUnf,VarDeliUnf;
					if(Inst.CapH==0){
						for (int i = 0; i < Inst.node; ++i) {
							if(dem[i]>Inst.CapaVehicle[v]){
								CapUnf=true;
								upper+=10000;
								if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
									resHub[Inst.StartVehicle[v]-Inst.Np]+=10000;
								}
								VarPickUnf.clear();
								for (size_t j = 0; j < VarPick.size(); j++){
									if(VarPick[j][0]==i)
										VarPickUnf.push_back(VarPick[j]);
								}
								for (size_t j = 0; j < VarDeli.size(); j++){
									if(VarDeli[j][0]==i)
										VarDeliUnf.push_back(VarDeli[j]);
								}
								if(!VarDeliUnf.empty())
									AddFeasCut(env,Inst,Inst.StartVehicle[v],w,VarDeliUnf,{},AddCuts,t);
								if(!VarPickUnf.empty())
									AddFeasCut(env,Inst,Inst.StartVehicle[v],w,VarPickUnf,{},AddCuts,t);
							}
						}
					}
					if(NodeSub.size()>1 && !CapUnf){
						Inst.NbNodeSubs+=NodeSub.size();
						Inst.NbSolvedSubs++;
						if((int)NodeSub.size()> Inst.MaxNode)
							Inst.MaxNode=NodeSub.size();
						if((int)NodeSub.size()<Inst.MinNode)
							Inst.MinNode=NodeSub.size();
						int tour=1;
						if(Inst.StartVehicle[v]>=Inst.Np){
							tour=min(Inst.TourHub,(int)NodeSub.size());
						}
						
						if(Inst.Bapcod==0){
							//cout<<"t "<<t <<" v "<<v<<" sig "<< MasterCplex.getValue(sigma[t][v])<<" "<<Inst.WorkVehicle[v]<<endl;
							//cout<<NodeSub<<endl;
							
							GenWorkerModel(env, WorkerModel,Inst,u,x,NodeSub,dem, tour,Inst.WorkVehicle[v],Inst.CapaVehicle[v]);
							
							WorkerCplex.solve();
							
							if( WorkerCplex.getStatus()==IloAlgorithm::Optimal){
								res[s]=WorkerCplex.getObjValue();
								//cout<<"upper + "<<res[s]<<endl;
								upper+=res[s];
								if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
									resHub[Inst.StartVehicle[v]-Inst.Np]+=res[s];
								}
								tot+=res[s];
								if(Inst.Output!=""){
									std::ofstream outFile(Inst.Output, std::ios::app);
									vector<pair<int,int>> Arcs;
									for (int r = 0; r < Inst.TourVehicle[v]; ++r) {
										Arcs.clear();
										for (int i = 0; i < (int) NodeSub.size(); i++){
											for (int j = 0; j < (int) NodeSub.size(); j++){
												if (NodeSub[i] != NodeSub[j]) {
													value=WorkerCplex.getValue(x[NodeSub[i]][NodeSub[j]][r]);
													if(value >1-epsi)
														Arcs.push_back({NodeSub[i],NodeSub[j]});
												}
											}
										}
										if(!Arcs.empty()){
											int k1,k=0;
											int currentnode=Inst.StartVehicle[v];
											outFile <<"Period "<<t<<" Vehicle "<<v<<endl;
											if(currentnode< Inst.Np)
												outFile <<"P"<<currentnode;
											else
												outFile <<"H"<<currentnode-Inst.Np;
											while(k<(int) NodeSub.size()){
												k1=0;
												while(Arcs[k1].first != currentnode)
													k1++;
												if(Arcs[k1].second < Inst.Np)
													outFile <<" P"<<Arcs[k1].second;
												else if(Arcs[k1].second < Inst.Np+Inst.Nh)
													outFile <<" H"<<Arcs[k1].second-Inst.Np;
												else if(Arcs[k1].second < Inst.Np+Inst.Nh+Inst.Nc)
													outFile <<" C"<<Arcs[k1].second-(Inst.Np+Inst.Nh);
												else
													outFile <<" H"<<Arcs[k1].second-(Inst.Np+Inst.Nh+Inst.Nc);
												currentnode=Arcs[k1].second;
												k++;
											}
											outFile <<endl;
										}
									}
									outFile.close();
								}
							}else if(WorkerCplex.getStatus()==IloAlgorithm::Infeasible){
								res[s]=-1;
								
								upper+=10000;
								if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
									resHub[Inst.StartVehicle[v]-Inst.Np]+=10000;
								}
								
							}else {  // For all other cases
								std::cerr << "Subproblem Solving failed" << std::endl;
								res[s]=-2;  // Optionally return -1 to indicate failure
							}
						}else{
							#ifdef USE_BAP
								res[s]=-2;
								if (NodeSub.size()==2){
									assert(dem[NodeSub[1]]<=Inst.CapaVehicle[v]);
									if(Inst.dist[NodeSub[0]][NodeSub[1]]*2 <= Inst.WorkVehicle[v] && dem[NodeSub[1]]<=Inst.CapaVehicle[v]){
										res[s]=Inst.dist[NodeSub[0]][NodeSub[1]]*2;
										
										upper+=res[s];
										if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
											resHub[Inst.StartVehicle[v]-Inst.Np]+=res[s];
										}
										
										tot+=res[s];
									}else{
										res[s]=-1;
										
										upper+=10000;
										if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
											resHub[Inst.StartVehicle[v]-Inst.Np]+=10000;
										}
									}

								}
								if (NodeSub.size()==3){
									if(dem[NodeSub[1]]+dem[NodeSub[2]]<=Inst.CapaVehicle[v]){
										if(Inst.dist[NodeSub[0]][NodeSub[1]]+Inst.dist[NodeSub[1]][NodeSub[2]]+Inst.dist[NodeSub[0]][NodeSub[2]] <= Inst.WorkVehicle[v]){
											res[s]=Inst.dist[NodeSub[0]][NodeSub[1]]+Inst.dist[NodeSub[1]][NodeSub[2]]+Inst.dist[NodeSub[0]][NodeSub[2]];
											
											upper+=res[s];
											if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
												resHub[Inst.StartVehicle[v]-Inst.Np]+=res[s];
											}
											
											tot+=res[s];
										}else{
											res[s]=-1;
											upper+=10000;
											if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
												resHub[Inst.StartVehicle[v]-Inst.Np]+=10000;
											}
										}
									}else{
										if(Inst.dist[NodeSub[0]][NodeSub[1]]*2+Inst.dist[NodeSub[0]][NodeSub[2]]*2 <= Inst.WorkVehicle[v]){
											res[s]=Inst.dist[NodeSub[0]][NodeSub[1]]*2+Inst.dist[NodeSub[0]][NodeSub[2]]*2;
											upper+=res[s];
											if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
												resHub[Inst.StartVehicle[v]-Inst.Np]+=res[s];
											}

											tot+=res[s];
										}else{
											res[s]=-1;
											upper+=10000;
											if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
												resHub[Inst.StartVehicle[v]-Inst.Np]+=10000;
											}
										}
									}
								}
								
								if(NodeSub.size()>7){
									IloEnv env2;	
									IloArray<IloArray<IloNumVar>> u2(env2, Inst.node);
									IloArray<IloArray<IloArray<IloBoolVar>>> x2(env2, Inst.node);
									IloModel WorkerModel2(env2);
									//cout<<ProblemHubDeli[h]<<endl;
									//cout<<DemandsProblemHubDeli[h]<<endl;
									createWorkerModelFeas(env2, WorkerModel2,Inst,u2,x2);
									IloCplex WorkerCplex2(WorkerModel2);
									WorkerCplex2.setParam(IloCplex::Param::Threads, 1);
									WorkerCplex2.setParam(IloCplex::Param::MIP::Display, 0);
									WorkerCplex2.setParam(IloCplex::Param::TimeLimit, 5);
									WorkerCplex2.setOut(env2.getNullStream());  // Suppress log output
									WorkerCplex2.setWarning(env2.getNullStream());
									WorkerCplex2.setParam(IloCplex::Param::Emphasis::MIP, IloCplex::MIPEmphasisFeasibility);
									WorkerCplex2.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);

									GenWorkerModel(env2, WorkerModel2,Inst,u2,x2,NodeSub,dem, tour,Inst.WorkVehicle[v],Inst.CapaVehicle[v]);
									
									WorkerCplex2.solve();
									if(WorkerCplex2.getStatus()==IloAlgorithm::Infeasible){
										res[s]=-1;
										
										upper+=10000;
										if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
											resHub[Inst.StartVehicle[v]-Inst.Np]+=10000;
										}
										
									}
								}
								if(res[s]==-2){
									vrpstw::Loader loader;
									BcInitialisation bapcodInit("../config/bc.cfg");
									//BcInitialisation bapcodInit("/home/pmontalbano/Short-Circuit-Benders/config/bc.cfg");

									vector<int> xCoord,yCoord,demandbap,DistDepot;
									for (int i = 0; i < (int) NodeSub.size(); i++)
									{
										xCoord.push_back(Inst.x_all[NodeSub[i]]);
										yCoord.push_back(Inst.y_all[NodeSub[i]]);
										demandbap.push_back(dem[NodeSub[i]]);
										DistDepot.push_back(Inst.dist[Inst.StartVehicle[v]][NodeSub[i]]);
									}
									assert(demandbap[0]==0);
									pair<float,vector<vector<int>>> ResTour;
									vector<pair<int,vector<int>>> Resultat;
									ResTour=SolveWithBapcod(bapcodInit,loader,tour,Inst.CapaVehicle[v],Inst.WorkVehicle[v],(int)NodeSub.size(),xCoord,yCoord,demandbap,DistDepot,Resultat);
									res[s]=ResTour.first;
									if(res[s]> Inst.WorkVehicle[v] && Inst.NoMaxWork==0)
										res[s]=-1;
									if(res[s]==-1){
										
										upper+=10000;
										if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
											resHub[Inst.StartVehicle[v]-Inst.Np]+=10000;
										}
										
									}else{
										
										upper+=res[s];
										if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
											resHub[Inst.StartVehicle[v]-Inst.Np]+=res[s];
										}
										
										tot+=res[s];
									}
								}
							#endif
						}
						
						/*if (res[s] >-1 ) {  // If the status is optimal
							
							//cout<<"Real Cost "<<res[s]<< " "<<Inst.WorkVehicle[v]<<endl;
							//WorkerCplex.exportModel("filework.lp");
							for (int i = 0; i < (int) NodeSub.size(); i++)
							{
								for (int j = 0; j < (int) NodeSub.size(); j++){
									if (NodeSub[i] != NodeSub[j]) {
										for (int r = 0; r < Inst.TourVehicle[v]; ++r) {
											value=WorkerCplex.getValue(x[NodeSub[i]][NodeSub[j]][r]);
											if(value >1-epsi)
												cout<<x[NodeSub[i]][NodeSub[j]][r]<<" "<<Inst.dist[NodeSub[i]][NodeSub[j]]<<endl;
												//cout<<x[NodeSub[i]][NodeSub[j]][r]<<"_"<<t<<"_"<<v<<endl;
										}
									}
								}
							}
						} */
						//cout<<"t "<<t <<" v "<<v<<" sig "<< MasterCplex.getValue(sigma[t][v])<<" "<<res[s]<<endl;
						//cout<<NodeSub<<endl;
						int MoreThanOneTour=false;
						if(s==1 && TotalDem.first>Inst.CapaVehicle[v])
							MoreThanOneTour=true;
						if(s==0 && TotalDem.second>Inst.CapaVehicle[v])
							MoreThanOneTour=true;
						if(MoreThanOneTour  && Inst.AddImprove>=1 && Inst.FeasFirst<1){
							vector<int> dem2;
							for (size_t i = 0; i < dem.size(); i++){
								dem2.push_back(1);
							}
							GenWorkerModel(env, WorkerModel,Inst,u,x,NodeSub,dem2, 1,Inst.WorkVehicle[v],Inst.CapaVehicle[v]);
							WorkerCplex.solve();
							if( WorkerCplex.getStatus()==IloAlgorithm::Optimal){
								if(res[s]==-1 && NodeSub.size()==3 && s==1){
									for (int t = 0; t < Inst.Nt; ++t) {
										for (int vv = 0; vv < Inst.Nv; vv++){
											if(Inst.StartVehicle[vv]==Inst.StartVehicle[v]){
												IloExpr expr(env);
												for (int k = 0; k < Inst.Nk; ++k) {
													for (int c = 0; c < Inst.Nc; ++c) {
														if(Inst.demands[c][k]>0){
															if( Inst.stocks[NodeSub[1]][k] >= Inst.demands[c][k])
																expr += w[NodeSub[1]][t][v][k][c] * Inst.demands[c][k] * Inst.Psize[k];
															if(Inst.stocks[NodeSub[2]][k] >= Inst.demands[c][k])
																expr += w[NodeSub[2]][t][v][k][c] * Inst.demands[c][k] * Inst.Psize[k];
														}
													}
												}
												AddCuts.add(expr <= Inst.CapaHub);
												expr.end();
											}
										}
									}
								}
								resV2[s]=WorkerCplex.getObjValue();
								if(Inst.AddImprove==1 && MasterCplex.getValue(sigma[t][v],sol)!=resV2[s]){
									if(s==1){
										AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,resV2[s],PickAndDel.first,{},AddCuts,t,yr);
									}else{
										AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,resV2[s],PickAndDel.second,{},AddCuts,t,yr);
									}
								}
							}else if(WorkerCplex.getStatus()==IloAlgorithm::Infeasible){
								resV2[s]=-1;
								if(s==1){
									ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,PickAndDel.first,{},AddCuts,t,yr);
								}else{
									ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,PickAndDel.second,{},AddCuts,t,yr);
								}
								resV2[s]=0;
							}
						}
						if(res[s]==-1){							
							if(s==1){
								if(Inst.ImprovedCut==0 || TotalDem.first>Inst.CapaVehicle[v]){
									if(Inst.StrenghtenFeasCut==0){
										AddFeasCut(env,Inst,Inst.StartVehicle[v],w,VarPick,{},AddCuts,t);
									}else{
										vector<bool> MinExpl = StrengthenFeasCut(NodeSub,Inst,dem,tour,Inst.WorkVehicle[v],Inst.CapaVehicle[v],false,VarPick);
										vector<array<int,3>> MinSet ;
										for (size_t i = 0; i < VarPick.size(); i++){
											if(MinExpl[i])
												MinSet.push_back(VarPick[i]);
										}
										AddFeasCut(env,Inst,Inst.StartVehicle[v],w,MinSet,{},AddCuts,t);
									}
								}else{
									if(Inst.StrenghtenFeasCut==0){
										ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,PickAndDel.first,{},AddCuts,t,yr);
									}
									else{
										vector<bool> MinExpl = StrengthenFeasCut(NodeSub,Inst,dem,tour,Inst.WorkVehicle[v],Inst.CapaVehicle[v],true,{});
										vector<int> MinSet;
										for (size_t i = 0; i < PickAndDel.first.size(); i++){
											if(MinExpl[i])
												MinSet.push_back(PickAndDel.first[i]);
										}
										ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,MinSet,{},AddCuts,t,yr);
									}
								}
								if(Inst.MoreCuts==1)
										AddMoreFeasCut(env,Inst,Inst.StartVehicle[v],w,VarPick,{},AddCuts);
							}else{
								if(Inst.ImprovedCut==0 || TotalDem.second > Inst.CapaVehicle[v]){
									if(Inst.StrenghtenFeasCut==0){
										AddFeasCut(env,Inst,Inst.StartVehicle[v],w,VarDeli,{},AddCuts,t);
									}else{
										vector<bool> MinExpl = StrengthenFeasCut(NodeSub,Inst,dem,tour,Inst.WorkVehicle[v],Inst.CapaVehicle[v],false,VarDeli);
										vector<array<int,3>> MinSet ;
										for (size_t i = 0; i < VarDeli.size(); i++){
											if(MinExpl[i])
												MinSet.push_back(VarDeli[i]);
										}
										AddFeasCut(env,Inst,Inst.StartVehicle[v],w,MinSet,{},AddCuts,t);
									}
								}else{
									if(Inst.StrenghtenFeasCut==0){
										ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,PickAndDel.second,{},AddCuts,t,yr);
									}else{
										vector<bool> MinExpl = StrengthenFeasCut(NodeSub,Inst,dem,tour,Inst.WorkVehicle[v],Inst.CapaVehicle[v],true,{});
										vector<int> MinSet;
										for (size_t i = 0; i < PickAndDel.second.size(); i++){
											if(MinExpl[i])
												MinSet.push_back(PickAndDel.second[i]);
										}
										ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,MinSet,{},AddCuts,t,yr);
									}
								if(Inst.MoreCuts==1)
										AddMoreFeasCut(env,Inst,Inst.StartVehicle[v],w,{},VarDeli,AddCuts);
							}
							res[s]=0;
							}
						}
					}
				}
				//cout<<tot<<" "<<epsi<<" "<<Inst.WorkVehicle[v]<<endl;
				if(tot>epsi){
					/*if(Inst.StartVehicle[v] < Inst.Np){
						Inst.nbLengthTourProd++;
						Inst.LengthTourProd+=tot*100/Inst.WorkProd;
					}else{
						Inst.nbLengthTourHub++;
						Inst.LengthTourHub+=tot*100/Inst.WorkHub;
					}*/
				
					if(tot>Inst.WorkVehicle[v]){
						
						upper+=10000;
						if(Inst.YannickT>=1 && Inst.StartVehicle[v]>=Inst.Np && NodeSub.size()>1 ){
							resHub[Inst.StartVehicle[v]-Inst.Np]+=10000;
						}
						//cout<<"tot "<<tot<<" "<<Inst.WorkVehicle[v]<<endl;
						if(Inst.ImprovedCut==0 || TotalDem.first>Inst.CapaVehicle[v] || TotalDem.second > Inst.CapaVehicle[v]){
							AddFeasCut(env,Inst,Inst.StartVehicle[v],w,VarPick,VarDeli,AddCuts,t);
							if(Inst.FeasFirst<1)
								AddOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,tot,VarPick,VarDeli,AddCuts,t);
						}else{
							ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,PickAndDel.first,PickAndDel.second,AddCuts,t,yr);
							if(Inst.FeasFirst<1)	
								AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,tot,PickAndDel.first,PickAndDel.second,AddCuts,t,yr);
						}
						if(Inst.MoreCuts==1){
							AddMoreFeasCut(env,Inst,Inst.StartVehicle[v],w,VarPick,VarDeli,AddCuts);											
							AddMoreOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,tot,VarPick,VarDeli,AddCuts);
						}
						if(Inst.PartialCut==1){
							if(res[0]>0 && res[1]>0){
								if(Inst.ImprovedCut==0 || TotalDem.first>Inst.CapaVehicle[v])
									AddOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[1],VarPick,{},AddCuts,t);
								else
									AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,res[1],PickAndDel.first,{},AddCuts,t,yr);
								if(Inst.MoreCuts==1)
									AddMoreOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[1],VarPick,{},AddCuts);
								if(Inst.ImprovedCut==0 || TotalDem.second>Inst.CapaVehicle[v])
									AddOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[0],VarDeli,{},AddCuts,t);
								else
									AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,res[0],PickAndDel.second,{},AddCuts,t,yr);
								if(Inst.MoreCuts==1)
									AddMoreOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[0],{},VarDeli,AddCuts);
							}
						}
					}else if(Inst.FeasFirst<1 && ((sol==-1 && tot>MasterCplex.getValue(sigma[t][v])+epsi) || (sol!=-1 && tot>MasterCplex.getValue(sigma[t][v],sol)+epsi))){
						if(Inst.ImprovedCut==0 || TotalDem.first>Inst.CapaVehicle[v] || TotalDem.second > Inst.CapaVehicle[v])
							AddOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,tot,VarPick,VarDeli,AddCuts,t);
						else
							AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,tot,PickAndDel.first,PickAndDel.second,AddCuts,t,yr);
						if(Inst.MoreCuts==1)
							AddMoreOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,tot,VarPick,VarDeli,AddCuts);
						if(Inst.PartialCut==1){
							if(res[0]>0 && res[1]>0){
								if(Inst.ImprovedCut==0 || TotalDem.first>Inst.CapaVehicle[v])
									AddOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[1],VarPick,{},AddCuts,t);
								else
									AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,res[1],PickAndDel.first,{},AddCuts,t,yr);
								if(Inst.MoreCuts==1)
									AddMoreOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[1],VarPick,{},AddCuts);
								if(Inst.ImprovedCut==0 || TotalDem.second>Inst.CapaVehicle[v])
									AddOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[0],VarDeli,{},AddCuts,t);
								else
									AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,res[0],PickAndDel.second,{},AddCuts,t,yr);
								if(Inst.MoreCuts==1)
									AddMoreOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[0],{},VarDeli,AddCuts);
							}
						}						
					}
					if(Inst.AddImprove>=1 && (resV2[0]>0 && resV2[1]>0)){
						//assert(resV2[0]+resV2[1]<=tot);
						if(resV2[0]+resV2[1]>Inst.WorkVehicle[v]){
							ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,PickAndDel.first,PickAndDel.second,AddCuts,t,yr);
							AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,resV2[0]+resV2[1],PickAndDel.first,PickAndDel.second,AddCuts,t,yr);
						}else if(Inst.AddImprove==1 && ((sol ==-1 && resV2[0]+resV2[1]>MasterCplex.getValue(sigma[t][v])+epsi) || (sol!=-1 && resV2[0]+resV2[1]>MasterCplex.getValue(sigma[t][v],0)+epsi))){
							AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,resV2[0]+resV2[1],PickAndDel.first,PickAndDel.second,AddCuts,t,yr);
						}

					}
				}
			}
		}
				  
		if(Inst.YannickT>=1 && Inst.FeasFirst<1 && (Inst.YannickT==5 ||Inst.MoreSol==1 || MasterCplex.getObjValue()== MasterCplex.getObjValue(sol))){
			int opt1,opt2, SigmaHub;
			vector<int> TourCostPick,TourCostDeli;
			vector<vector<int>> TourPick, TourDeli;
			
			for (int h = 0; h < Inst.Nh; h++){
				opt1=0;
				opt2=0;
				TourPick={};
				TourDeli={};
				TourCostPick={};
				TourCostDeli={};
				SigmaHub=0;
				for (size_t v = 0; v < Inst.Vehicles[Inst.Np+h].size(); v++){
					if(sol==-1){
						SigmaHub+=MasterCplex.getValue(sigma[t][Inst.Vehicles[Inst.Np+h][v]]);
					}
					else{
						SigmaHub+=MasterCplex.getValue(sigma[t][Inst.Vehicles[Inst.Np+h][v]],sol);
					}
				}
				/*if(SigmaHub>0)
					assert(!VarDeliHub[h].empty() || !VarPickHub[h].empty());*/
				if(SigmaHub<resHub[h] && (Inst.YannickT!=3 || resHub[h]>=10000)){
					if(VarDeliHub[h].size()>0){		
						bool OneVertex=true;
						ProblemHubDeli[h].push_back(h+Inst.Np);
						DemandsProblemHubDeli[h].push_back(0);
						for (size_t i = 0; i < VarDeliHub[h].size(); i++)
						{
							//cout<<VarDeliHub[h][i][0]<<" "<<VarDeliHub[h][i][1]<<" "<<VarDeliHub[h][i][2]<<endl;
							ProblemHubDeli[h].push_back(VarDeliHub[h][i][0]);
							if(ProblemHubDeli[h][1]!=ProblemHubDeli[h].back())
								OneVertex=false;
							DemandsProblemHubDeli[h].push_back(Inst.demands[VarDeliHub[h][i][2]][VarDeliHub[h][i][1]]*Inst.Psize[VarDeliHub[h][i][1]]);
						}
						//int tour=ceil(accumulate(DemandsProblemHubDeli[h].begin(), DemandsProblemHubDeli[h].end(), 0)/(double)Inst.CapaHub);
						int tour=ProblemHubDeli[h].size();
						if(!OneVertex){
							if(Inst.Bapcod>=1){
								#ifdef USE_BAP

									vrpstw::Loader loader;
									BcInitialisation bapcodInit(Inst.configFile);
									vector<int> xCoord,yCoord,demandbap,DistDepot;
									for (int i = 0; i < (int) ProblemHubDeli[h].size(); i++)
									{
										xCoord.push_back(Inst.x_all[ProblemHubDeli[h][i]]);
										yCoord.push_back(Inst.y_all[ProblemHubDeli[h][i]]);
										demandbap.push_back(DemandsProblemHubDeli[h][i]);
										DistDepot.push_back(Inst.dist[Inst.Np+h][ProblemHubDeli[h][i]]);
									}
									assert(demandbap[0]==0);
									/*cout<<"DELIVERY PROBLEM"<<tour<<endl;
									cout<<ProblemHubDeli[h]<<endl;
									cout<<demandbap<<endl;*/
									pair<float,vector<vector<int>>> ResTour;	
									vector<pair<int,vector<int>>> Resultat;
									ResTour=SolveWithBapcod(bapcodInit,loader,min(tour,Inst.Nvh*(Inst.Np+Inst.Nh-1)),Inst.CapaHub,Inst.WorkHub,(int)demandbap.size(),xCoord,yCoord,demandbap,DistDepot,Resultat);
									if(ResTour.first!=-1){
										opt1=ResTour.first;

										for (size_t i = 1; i < Resultat.size(); i++){
											TourDeli.push_back(Resultat[i].second);
											TourDeli.back().erase(TourDeli.back().begin());
											TourCostDeli.push_back(Resultat[i].first);
										}
										
										/*TourDeli=ResTour.second;
										
										bool BapCodFail=false;
										TourCostDeli={};
										for (size_t i = 0; i < TourDeli.size(); i++){
											TourDeli[i].pop_back();
											for (size_t j = 0; j < TourDeli[i].size(); j++){
												for (size_t  l= 0; l < TourDeli[i].size(); l++){
													if(l!=j && TourDeli[i][l]==TourDeli[i][j])
														BapCodFail=true;
												}
											}
											if(!BapCodFail || 1==1){
												TourCostDeli.push_back(0);
												TourCostDeli.back()+=Inst.dist[ProblemHubDeli[h][0]][ProblemHubDeli[h][TourDeli[i][0]]];
												for (size_t j = 1; j < TourDeli[i].size(); j++){
													TourCostDeli.back()+=Inst.dist[ProblemHubDeli[h][TourDeli[i][j]]][ProblemHubDeli[h][TourDeli[i][j-1]]];
												}
												TourCostDeli.back()+=Inst.dist[ProblemHubDeli[h][0]][ProblemHubDeli[h][TourDeli[i].back()]];
											}else{
												IloEnv env2;	
												IloArray<IloArray<IloNumVar>> u2(env2, Inst.node);
												IloArray<IloArray<IloArray<IloBoolVar>>> x2(env2, Inst.node);
												IloModel WorkerModel2(env2);
												//cout<<ProblemHubDeli[h]<<endl;
												//cout<<DemandsProblemHubDeli[h]<<endl;
												vector<int> TempProblemHubDeli;
												vector<int> AlreadyIn;
												TempProblemHubDeli.push_back(ProblemHubDeli[h][0]);
												for (size_t j = 0; j < TourDeli[i].size(); j++)
												{
													if (std::find(AlreadyIn.begin(), AlreadyIn.end(), TourDeli[i][j]) == AlreadyIn.end()) {
														TempProblemHubDeli.push_back(ProblemHubDeli[h][TourDeli[i][j]]); 
														AlreadyIn.push_back(TourDeli[i][j]);
													}
												}
												GenWorkerModelHub(env2, WorkerModel2,Inst,u2,x2,TempProblemHubDeli,DemandsProblemHubDeli[h], 1,Inst.WorkHub,Inst.CapaHub);

												IloCplex WorkerCplex2(WorkerModel2);
												WorkerCplex2.setParam(IloCplex::Param::Threads, 1);
												WorkerCplex2.setParam(IloCplex::Param::MIP::Display, 0);
												WorkerCplex2.setOut(env.getNullStream());  // Suppress log output
												WorkerCplex2.setWarning(env.getNullStream());
												WorkerCplex2.setParam(IloCplex::Param::TimeLimit, 5);
												WorkerCplex2.solve();
												assert(WorkerCplex2.getStatus() == IloAlgorithm::Optimal);
												TourDeli[i]=AlreadyIn;
												TourCostDeli.push_back(WorkerCplex2.getObjValue());
												WorkerCplex2.end();
												WorkerModel2.end();
												env2.end();
											}
										}*/
									}
									else
										opt1=10000;
									/*cout<<opt1<<endl;
									cout<<"TOuR DELi "<<endl;
									cout<<TourDeli<<endl;
									cout<<TourCostDeli<<endl;
									cout<<"DELI 2"<<endl;
									cout<<TourDeli2<<endl;
									cout<<TourCostDeli2<<endl;
									assert(TourCostDeli2==TourCostDeli);
									assert(TourDeli.size()==TourDeli2.size());
									assert(std::equal(TourDeli.begin(), TourDeli.end(), TourDeli2.begin(), TourDeli2.end(),[](const auto& a, const auto& b) { return a.size() == b.size(); }));*/
									assert(opt1==10000 || (accumulate(TourCostDeli.begin(), TourCostDeli.end(), 0)-opt1<0.1 && accumulate(TourCostDeli.begin(), TourCostDeli.end(), 0)-opt1>-0.1));								
								#endif
							}
							else{
								IloEnv env2;	
								IloArray<IloArray<IloNumVar>> u2(env2, Inst.node);
								IloArray<IloArray<IloArray<IloBoolVar>>> x2(env2, Inst.node);
								IloModel WorkerModel2(env2);
								//cout<<ProblemHubDeli[h]<<endl;
								//cout<<DemandsProblemHubDeli[h]<<endl;
								GenWorkerModelHub(env2, WorkerModel2,Inst,u2,x2,ProblemHubDeli[h],DemandsProblemHubDeli[h], tour,Inst.WorkHub,Inst.CapaHub);
								IloCplex WorkerCplex2(WorkerModel2);
								WorkerCplex2.setParam(IloCplex::Param::Threads, 1);
								WorkerCplex2.setParam(IloCplex::Param::MIP::Display, 0);
								WorkerCplex2.setOut(env.getNullStream());  // Suppress log output
								WorkerCplex2.setWarning(env.getNullStream());
								WorkerCplex2.setParam(IloCplex::Param::TimeLimit, 5);
								WorkerCplex2.solve();
								if (WorkerCplex2.getStatus() == IloAlgorithm::Optimal){
									opt1=WorkerCplex2.getObjValue();
									for (int r = 0; r < tour; ++r) {
										if(!TourDeli.back().empty()){
											TourDeli.push_back({});
											TourCostDeli.push_back(0);
										}
										for (size_t i = 0; i < ProblemHubDeli[h].size(); ++i) {
											for (size_t j = 0; j < ProblemHubDeli[h].size(); ++j) {
												if(i!=j){
													value=WorkerCplex2.getValue(x2[i][j][r]);
													if(value>1-epsi){
														TourDeli.back().push_back(j);
														TourCostDeli.back()+=Inst.dist[ProblemHubDeli[h][j]][ProblemHubDeli[h][i]];
													}
												}
											}
										}
									}
									assert(accumulate(TourCostDeli.begin(), TourCostDeli.end(), 0)-opt1<0.1 && accumulate(TourCostDeli.begin(), TourCostDeli.end(), 0)-opt1>-0.1 );
								}else{
									opt1=10000;
								}
								WorkerCplex2.end();
								WorkerModel2.end();
								env2.end();
							}
						}else{
							TourDeli.push_back({});
							TourCostDeli.push_back(0);
							for (size_t i = 0; i < ceil(accumulate(DemandsProblemHubDeli[h].begin(), DemandsProblemHubDeli[h].end(), 0)/(double)Inst.CapaHub); i++){
								TourDeli.back().push_back(1);
								TourDeli.push_back({});
								TourCostDeli.back()+=2*Inst.dist[Inst.Np+h][ProblemHubDeli[h][1]];
								TourCostDeli.push_back(0);
								opt1+=2*Inst.dist[Inst.Np+h][ProblemHubDeli[h][1]];
							}
						}
						
					}
					if(VarPickHub[h].size()>0){	
						bool OneVertex=true;
						ProblemHubPick[h].push_back(h+Inst.Np);
						DemandsProblemHubPick[h].push_back(0);
						vector <int> demandepernode(Inst.Np+Inst.Nh,0);
						vector <bool> split(Inst.Np+Inst.Nh,true),splitted;
						bool Feas=true;
						for (size_t i = 0; i < VarPickHub[h].size(); i++){
							//cout<<VarPickHub[h][i][0]<<" "<<VarPickHub[h][i][1]<<" "<<VarPickHub[h][i][2]<<endl;
							ProblemHubPick[h].push_back(VarPickHub[h][i][0]);
							if(ProblemHubPick[h][1]!=ProblemHubPick[h].back())
								OneVertex=false;
							DemandsProblemHubPick[h].push_back(Inst.demands[VarPickHub[h][i][2]][VarPickHub[h][i][1]]*Inst.Psize[VarPickHub[h][i][1]]);
							demandepernode[ProblemHubPick[h].back()]+=DemandsProblemHubPick[h].back();
							if(demandepernode[ProblemHubPick[h].back()]>Inst.CapaHub){
								split[ProblemHubPick[h].back()]=false;
								if(Inst.YannickT==4)
									Feas=false;
							}
						}
						
						//int tour=ceil(accumulate(DemandsProblemHubPick[h].begin(), DemandsProblemHubPick[h].end(), 0)/(double)Inst.CapaHub);
						int tour=ProblemHubPick[h].size();
						if(Feas){
							if(!OneVertex){
								if(Inst.Bapcod>=1){
									#ifdef USE_BAP
										vrpstw::Loader loader;
										BcInitialisation bapcodInit(Inst.configFile);
										vector<int> xCoord,yCoord,demandbap,DistDepot,nodeMaster,posinPick;
										if(Inst.YannickT==1){
											for (int i = 0; i < (int) ProblemHubPick[h].size(); i++){
												xCoord.push_back(Inst.x_all[ProblemHubPick[h][i]]);
												yCoord.push_back(Inst.y_all[ProblemHubPick[h][i]]);
												demandbap.push_back(DemandsProblemHubPick[h][i]);
												DistDepot.push_back(Inst.dist[Inst.Np+h][ProblemHubPick[h][i]]);
											}
										}else{
											xCoord.push_back(Inst.x_all[Inst.Np+h]);
											yCoord.push_back(Inst.y_all[Inst.Np+h]);
											DistDepot.push_back(0);
											demandbap.push_back(0);
											nodeMaster.push_back(Inst.Np+h);
											splitted.push_back(false);
											posinPick.push_back(0);
											for (int i = 0; i < Inst.Np +Inst.Nh; i++){
												if(demandepernode[i]>0 && i!= Inst.Np+h){
													if(split[i]){
														xCoord.push_back(Inst.x_all[i]);
														yCoord.push_back(Inst.y_all[i]);
														nodeMaster.push_back(i);
														DistDepot.push_back(Inst.dist[Inst.Np+h][i]);
														demandbap.push_back(demandepernode[i]);
														splitted.push_back(false);
														posinPick.push_back(0);
													}else{
														for (int j = 0; j < (int) ProblemHubPick[h].size(); j++){
															if(ProblemHubPick[h][j]==i){
																xCoord.push_back(Inst.x_all[ProblemHubPick[h][j]]);
																yCoord.push_back(Inst.y_all[ProblemHubPick[h][j]]);
																demandbap.push_back(DemandsProblemHubPick[h][j]);
																DistDepot.push_back(Inst.dist[Inst.Np+h][ProblemHubPick[h][j]]);
																splitted.push_back(true);
																nodeMaster.push_back(i);
																posinPick.push_back(j);
															}
														}
													}
												}
											}
										}
										/*cout<<"Pick PROBLEM"<<tour<<endl;
										cout<<ProblemHubPick[h]<<endl;
										cout<<"coord "<<xCoord<<" "<<yCoord<<endl;
										cout<<"dembap "<<demandbap<<endl;
										cout<<"dem nodes "<<demandepernode<<endl;
										cout<<"demprob "<<DemandsProblemHubPick[h]<<endl;*/
										opt2=0;
										//cout<<accumulate(demandepernode.begin(), demandepernode.end(), 0)<<" "<<Inst.CapaHub<<endl;
										//cout<<(float)accumulate(demandepernode.begin(), demandepernode.end(), 0)/Inst.CapaHub<<" "<<demandepernode.size()-2<<endl;
										
										
										pair<float,vector<vector<int>>> ResTour;	
										vector<pair<int,vector<int>>> Resultat;
										ResTour=SolveWithBapcod(bapcodInit,loader,min(tour,Inst.Nvh*(Inst.Np+Inst.Nh-1)),Inst.CapaHub,Inst.WorkHub,(int)demandbap.size(),xCoord,yCoord,demandbap,DistDepot,Resultat);
										//cout<<ResTour.second<<" "<<ProblemHubPick[h]<<" "<<endl;
										if(ResTour.first!=-1){
											
											if(Inst.YannickT==1){
												for (size_t i = 1; i < Resultat.size(); i++){
													TourPick.push_back(Resultat[i].second);
													TourPick.back().erase(TourPick.back().begin());
													TourCostPick.push_back(Resultat[i].first);
												}
												//TourPick=ResTour.second;
												opt2=ResTour.first;
												/*TourCostPick={};
												for (size_t i = 0; i < TourPick.size(); i++){
													TourPick[i].pop_back();
													if(TourPick[i].size() > 1 && TourPick[i][0]==TourPick[i].back())
														TourPick[i].pop_back();
													TourCostPick.push_back(0);
													TourCostPick.back()+=Inst.dist[ProblemHubPick[h][0]][ProblemHubPick[h][TourPick[i][0]]];
													for (size_t j = 1; j < TourPick[i].size(); j++){
														TourCostPick.back()+=Inst.dist[ProblemHubPick[h][TourPick[i][j]]][ProblemHubPick[h][TourPick[i][j-1]]];
													}
													TourCostPick.back()+=Inst.dist[ProblemHubPick[h][0]][ProblemHubPick[h][TourPick[i].back()]];
												}*/
											}else{
												opt2=ResTour.first;
												TourCostPick={};
												for (size_t i = 1; i < Resultat.size(); i++){
													TourPick.push_back({});
													for (size_t j = 1; j < Resultat[i].second.size(); j++){
														if(!splitted[Resultat[i].second[j]]){
															for (size_t l = 0; l < ProblemHubPick[h].size(); l++){
																if(ProblemHubPick[h][l]==nodeMaster[Resultat[i].second[j]]){
																	TourPick.back().push_back(l);
																}										
															}
														}else{
															TourPick.back().push_back(posinPick[Resultat[i].second[j]]);
														}
													}
													TourCostPick.push_back(Resultat[i].first);
												}
												/*for (size_t i = 0; i < ResTour.second.size(); i++){	
													TourCostPick.push_back(0);
													TourPick.push_back({});
													if(!ResTour.second[i].empty()){
														ResTour.second[i].pop_back();
														if(ResTour.second[i].size() > 1 && ResTour.second[i][0]==ResTour.second[i].back())
															ResTour.second[i].pop_back();
													}
													for (size_t j = 0; j < ResTour.second[i].size(); j++){	
														if(j==0)
															TourCostPick.back()+=Inst.dist[nodeMaster[j]][nodeMaster[ResTour.second[i][j]]];
														else
															TourCostPick.back()+=Inst.dist[nodeMaster[ResTour.second[i][j]]][nodeMaster[ResTour.second[i][j-1]]];
														if(!splitted[ResTour.second[i][j]]){
															for (size_t l = 0; l < ProblemHubPick[h].size(); l++){
																if(ProblemHubPick[h][l]==nodeMaster[ResTour.second[i][j]]){
																	TourPick.back().push_back(l);
																}										
															}
														}else{
															TourPick.back().push_back(posinPick[ResTour.second[i][j]]);
														}
													}
													TourCostPick.back()+=Inst.dist[nodeMaster[0]][ProblemHubPick[h][TourPick.back().back()]];
												}*/
											}
										}else
											opt2=10000;
										/*cout<<"TOUR PI "<<endl;
										cout<<TourPick<<endl;
										cout<<TourCostPick<<endl;
										cout<<opt2<<endl;*/
										/*cout<<"Pick 2"<<endl;
										cout<<TourPick2<<endl;
										cout<<TourCostPick2<<endl;
										assert(TourCostPick2==TourCostPick);
										assert(TourPick.size()==TourPick2.size());
										assert(std::equal(TourPick.begin(), TourPick.end(), TourPick2.begin(), TourPick2.end(),[](const auto& a, const auto& b) { return a.size() == b.size(); }));*/
										assert(opt2==10000 || (accumulate(TourCostPick.begin(), TourCostPick.end(), 0)-opt2<0.1 && accumulate(TourCostPick.begin(), TourCostPick.end(), 0)-opt2>-0.1));								
									#endif
								}else{
									IloEnv env2;	
									IloArray<IloArray<IloNumVar>> u2(env2, Inst.node);
									IloArray<IloArray<IloArray<IloBoolVar>>> x2(env2, Inst.node);
									IloModel WorkerModel2(env2);
									/*cout<<"PICK PROBLEM"<<endl;
									cout<<ProblemHubPick[h]<<endl;
									cout<<DemandsProblemHubPick[h]<<endl;*/
									GenWorkerModelHub(env2, WorkerModel2,Inst,u2,x2,ProblemHubPick[h],DemandsProblemHubPick[h], tour,Inst.WorkHub,Inst.CapaHub);
									IloCplex WorkerCplex2(WorkerModel2);
									WorkerCplex2.setParam(IloCplex::Param::Threads, 1);
									WorkerCplex2.setParam(IloCplex::Param::MIP::Display, 0);
									WorkerCplex2.setOut(env.getNullStream());  // Suppress log output
									WorkerCplex2.setWarning(env.getNullStream());
									WorkerCplex2.setParam(IloCplex::Param::TimeLimit, 20);
									WorkerCplex2.solve();
									if (WorkerCplex2.getStatus() == IloAlgorithm::Optimal){
										opt2=WorkerCplex2.getObjValue();
										for (int r = 0; r < tour; ++r) {
											if(!TourPick.back().empty()){
												TourPick.push_back({});
												TourCostPick.push_back(0);
											}
											for (size_t i = 0; i < ProblemHubPick[h].size(); ++i) {
												for (size_t j = 0; j < ProblemHubPick[h].size(); ++j) {
													if(i!=j){
														value=WorkerCplex2.getValue(x2[i][j][r]);
														if(value>1-epsi){
															TourPick.back().push_back(j);
															TourCostPick.back()+=Inst.dist[ProblemHubPick[h][j]][ProblemHubPick[h][i]];
														}
													}
												}
											}
										}
										assert(accumulate(TourCostPick.begin(), TourCostPick.end(), 0)-opt2<0.1 && accumulate(TourCostPick.begin(), TourCostPick.end(), 0)-opt2>-0.1);
									}else{
										opt2=10000;
									}
									
									WorkerCplex2.end();
									WorkerModel2.end();
									env2.end();
								}
							}else{
								TourPick.push_back({});
								TourCostPick.push_back(0);
								for (size_t i = 0; i < ceil(accumulate(DemandsProblemHubPick[h].begin(), DemandsProblemHubPick[h].end(), 0)/(double)Inst.CapaHub); i++){
									TourPick.back().push_back(1);
									TourPick.push_back({});
									TourCostPick.back()+=2*Inst.dist[Inst.Np+h][ProblemHubPick[h][1]];
									TourCostPick.push_back(0);
									opt2+=2*Inst.dist[Inst.Np+h][ProblemHubPick[h][1]];
								}
							}
						}else
							opt2=10000;
					}
					/*cout<<opt1 <<" "<<opt2<<" "<<resHub[h]<<endl;
					cout<<TourCostPick<<" "<<TourPick<<endl;
					cout<<TourCostDeli<<" "<<TourDeli<<endl;*/
					if(opt1+opt2< resHub[h] && opt1<10000 && opt2 < 10000){
						//cout<<opt1<<" "<<opt2<<" "<<resHub[h]<<endl;
						/*cout<<"PickTOur "<<TourPick<<endl;
						cout<<"Cors Tour "<<TourCostPick<<endl;
						cout<<"TourDelis "<<TourDeli<<endl;
						cout<<"Cost Tour "<<TourCostDeli<<endl;*/
						bool success=AddCutsFromHub(env,Inst,w,sigma,y,yr,AddCuts,opt1,opt2,TourCostPick,TourCostDeli,TourPick,TourDeli,ProblemHubPick[h],ProblemHubDeli[h],DemandsProblemHubDeli[h],DemandsProblemHubPick[h],VarPickHub[h],VarDeliHub[h],t,h);
						if(success){
							upper+=opt1+opt2-resHub[h];
						}
					}
				}
				ProblemHubDeli[h].clear();
				VarDeliHub[h].clear();
				DemandsProblemHubDeli[h].clear();
				ProblemHubPick[h].clear();
				VarPickHub[h].clear();
				DemandsProblemHubPick[h].clear();
				resHub[h]=0;
			}
		}
	}
	return upper;
}


void AddSigmmaCuts(IloEnv& env, IloCplex& MasterCplex, IloCplex& WorkerCplex, IloModel& WorkerModel,  MyInstance& Inst, int sol, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w, IloArray<IloArray<IloBoolVarArray>> y, IloArray<IloArray<IloNumVarArray>> yr, IloArray<IloArray<IloNumVar>> sigma, IloArray<IloArray<IloNumVar>>& u,IloArray<IloArray<IloArray<IloBoolVar>>>& x, IloConstraintArray& AddCuts){
	pair<vector<int>,vector<int>> PickAndDel,DemandPickAndDel(vector<int>(Inst.node,0),vector<int>(Inst.node,0));
	vector<array<int,3>> VarPick, VarDeli;
	pair<int,int> TotalDem;
	vector<int> NodeSub,dem;
	int chg;
	IloExpr expr(env);
	bool AlreadyAdded;
	vector<int> submin;
	vector<double> nbcommand;

	for (int t = 0; t < Inst.Nt; ++t) {
		for (int i : Inst.Pplus){
			if((i < Inst.Np && Inst.Prod_av[i][t]) || i >= Inst.Np){
				VarPick.clear();
				submin.clear();
				nbcommand.clear();
				for (int j : Inst.Cmoins){ 
					AlreadyAdded=false;
					for (int c = 0; c < Inst.Nc; ++c) {
						if(j==c+Inst.Np+Inst.Nh || (j >= Inst.Np+Inst.Nc+Inst.Nh && (i<Inst.Np || j!=Inst.PairHub[i-Inst.Np].second )) ){
							for (int k = 0; k < Inst.Nk; ++k) {	
								if(Inst.demands[c][k]>0 && (i >= Inst.Np || (Inst.stocks[i][k]>=Inst.demands[c][k] &&  Inst.dist[i][j]< 42)) && ((t >= Inst.DeliWindowsEar[c][k] -1 && t <= Inst.DeliWindowsLat[c][k]-1) || (j >= Inst.Np+Inst.Nc+Inst.Nh))){
									if(!AlreadyAdded){
										AlreadyAdded=true;
										VarPick.push_back({j,k,c});
										nbcommand.push_back(1);
									}else{
										nbcommand.back()++;
										if(Inst.SigmaCuts==1){
											VarPick.push_back({j,k,c});
										}
									}
								}
							}
						}
					}
				}
				for (size_t j = 0; j < VarPick.size(); j++){
					if(j==0 || VarPick[j][0]!=VarPick[j-1][0]){
						submin.push_back(Inst.dist[i][VarPick[j][0]]);
						for (size_t l = 0; l < VarPick.size(); l++){
							if( VarPick[j][0] != VarPick[l][0] && Inst.dist[VarPick[j][0]][VarPick[l][0]]<= submin.back())
								submin.back()=Inst.dist[VarPick[j][0]][VarPick[l][0]];
						}
					}
				}
				for(size_t v=0; v  < Inst.Vehicles[i].size();v++){
					//cout<<"Deli t "<<t<<" v "<<Inst.Vehicles[i][v]<<" "<<VarPick.size()<<" "<<submin<<endl;
					expr.clear();
					if(Inst.SigmaCuts==1){
						chg=0;					
						for (size_t l = 0; l < VarPick.size(); l++){
							if(l>0 && VarPick[l][0]!=VarPick[l-1][0])
								chg++;
							expr += (submin[chg]/nbcommand[chg]) * w[VarPick[l][0]][t][Inst.Vehicles[i][v]][VarPick[l][1]][VarPick[l][2]];
						}
					}else{
						assert(Inst.ImprovedCut>=1);
						assert(VarPick.size()==submin.size());
						for (size_t l = 0; l < VarPick.size(); l++){
							if(Inst.ImprovedCut==1)
								expr += submin[l] * y[VarPick[l][0]][t][Inst.Vehicles[i][v]];
							else{
								expr += submin[l] * yr[VarPick[l][0]][t][Inst.Vehicles[i][v]];
								if(Inst.SigmaCuts==3)
									AddCuts.add(yr[VarPick[l][0]][t][Inst.Vehicles[i][v]]*2*Inst.dist[i][VarPick[l][0]] <= sigma[t][Inst.Vehicles[i][v]]);
							}
						}
					}
					AddCuts.add(expr <= sigma[t][Inst.Vehicles[i][v]]);
				}
			}
			if(i >= Inst.Np){
				VarPick.clear();
				submin.clear();
				nbcommand.clear();
				for (int j = 0; j < Inst.Np; ++j){
					AlreadyAdded=false;
					for (int c = 0; c < Inst.Nc; ++c) {
						for (int k = 0; k < Inst.Nk; ++k) {	
							if(Inst.demands[c][k]>0 && Inst.stocks[j][k]>=Inst.demands[c][k]){
								if(!AlreadyAdded){
									AlreadyAdded=true;
									VarPick.push_back({j,k,c});
									nbcommand.push_back(1);
								}else{
									nbcommand.back()++;
									if(Inst.SigmaCuts==1){
										VarPick.push_back({j,k,c});
									}
								}
							}
						}
					}
				}
				for (size_t j = 0; j < VarPick.size(); j++){
					if(j==0 || VarPick[j][0]!=VarPick[j-1][0]){
						submin.push_back(Inst.dist[i][VarPick[j][0]]);
						for (size_t l = 0; l < VarPick.size(); l++){
							if( VarPick[j][0] != VarPick[l][0] && Inst.dist[VarPick[j][0]][VarPick[l][0]]<= submin.back())
								submin.back()=Inst.dist[VarPick[j][0]][VarPick[l][0]];
						}
					}
				}
				for(size_t v=0; v  < Inst.Vehicles[i].size();v++){
					//cout<<"t "<<t<<" v "<<Inst.Vehicles[i][v]<<" "<<VarPick.size()<<" "<<submin<<endl;
					expr.clear();
					if(Inst.SigmaCuts==1){
						chg=0;					
						for (size_t l = 0; l < VarPick.size(); l++){
							if(l>0 && VarPick[l][0]!=VarPick[l-1][0])
								chg++;
							expr += (submin[chg]/nbcommand[chg]) * w[VarPick[l][0]][t][Inst.Vehicles[i][v]][VarPick[l][1]][VarPick[l][2]];
						}
					}else{
						assert(Inst.ImprovedCut>=1);
						assert(VarPick.size()==submin.size());
						for (size_t l = 0; l < VarPick.size(); l++){
							if(Inst.ImprovedCut==1)
								expr += submin[l] * y[VarPick[l][0]][t][Inst.Vehicles[i][v]];
							else{
								expr += submin[l] * yr[VarPick[l][0]][t][Inst.Vehicles[i][v]];
								if(Inst.SigmaCuts==3)
									AddCuts.add(yr[VarPick[l][0]][t][Inst.Vehicles[i][v]]*2*Inst.dist[i][VarPick[l][0]] <= sigma[t][Inst.Vehicles[i][v]]);
							}
						}
					}
					AddCuts.add(expr <= sigma[t][Inst.Vehicles[i][v]]);
				}
			}
		}
	}	
}


std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (fgets(buffer, sizeof buffer, pipe) != NULL) 
            result += buffer;
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

int mainBend(MyInstance Inst, int BestUpper)
{
	try {
		
		cout << "Model the problem" << std::endl;
		IloEnv env;	
		IloModel MasterModel(env);
		IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w(env, Inst.node);
		IloArray<IloArray<IloNumVar>> sigma(env, Inst.Nt);
		IloArray<IloArray<IloBoolVarArray>> f(env, Inst.Np);
		IloArray<IloArray<IloBoolVarArray>> y(env, Inst.node);
		IloArray<IloArray<IloNumVarArray>> fr(env, Inst.Np);
		IloArray<IloArray<IloNumVarArray>> yr(env, Inst.node);
		createMasterModel(env,MasterModel,Inst,w,sigma,f,y,fr,yr);
		IloModel WorkerModel(env);
		IloArray<IloArray<IloNumVar>> u(env, Inst.node);
		IloArray<IloArray<IloArray<IloBoolVar>>> x(env, Inst.node);
		createWorkerModel(env,WorkerModel,Inst,u,x);
		
		IloCplex MasterCplex(MasterModel);
		IloCplex WorkerCplex(WorkerModel);
		
		MasterCplex.setOut(env.getNullStream());  // Suppress log output
		MasterCplex.setWarning(env.getNullStream());
		MasterCplex.setParam(IloCplex::Param::MIP::Display, 0);
		//MasterCplex.setParam(IloCplex::Param::RandomSeed, 12345);
		
		if(Inst.Gap>=1)
			MasterCplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, Inst.GAPlist[Inst.CurrGAP]);
		MasterCplex.setParam(IloCplex::Param::Threads, 1);
		if(Inst.MoreSol>1){
			MasterCplex.setParam(IloCplex::Param::MIP::Pool::Capacity, Inst.MoreSol);
			MasterCplex.setParam(IloCplex::Param::MIP::Pool::Replace, 2); 
		}
		
		WorkerCplex.setParam(IloCplex::Param::Threads, 1);
		WorkerCplex.setParam(IloCplex::Param::MIP::Display, 0);
		WorkerCplex.setOut(env.getNullStream());  // Suppress log output
		WorkerCplex.setParam(IloCplex::IloCplex::Param::MIP::Tolerances::Integrality, 1e-9);  // Integer feasibility tolerance
		WorkerCplex.setParam(IloCplex::Param::Simplex::Tolerances::Optimality, 1e-9);  // Optimality tolerance
		WorkerCplex.setParam(IloCplex::Param::Feasopt::Tolerance, 1e-9);
		WorkerCplex.setWarning(env.getNullStream());
		int maxIterations=10000,iter=0;
		pair<int,int> TotalDem;
		bool GetOut=false;
		vector<int> NodeSub,dem;
		pair<vector<int>,vector<int>> PickAndDel,DemandPickAndDel(vector<int>(Inst.node,0),vector<int>(Inst.node,0));
		IloConstraintArray AddCuts(env);
		
		float lower=0,upper=0,epsi = 1e-5;
		
		auto start = std::chrono::high_resolution_clock::now();
		if(Inst.ColdStart==1)
			MasterCplex.setParam(IloCplex::Param::Advance, 0);
		cout<<"Solve the Problem"<<endl;
		IloRange ObjCtr,ObjCtrLow;
		int sameLB=0;
		if(Inst.AddConstraintObj==1){
			IloExpr expr(env);
			if(Inst.NoObj==0){
				for (int i = 0; i < Inst.Np; ++i) {
					for (int k = 0; k < Inst.Nk; ++k) {
						if(Inst.stocks[i][k]>0){
							for (int j = 0; j < Inst.Nc; ++j) {	
								if(Inst.demands[j][k]>0){
									if(Inst.FReal==0)
										expr += Inst.dist[i][j+Inst.Np+Inst.Nh] * f[i][k][j];
									else
										expr += Inst.dist[i][j+Inst.Np+Inst.Nh] * fr[i][k][j];
								}
							}
						}
					}
				}
			}
			for (int t = 0; t < Inst.Nt; ++t) {
				for (int v = 0; v < Inst.Nv; ++v) {
					expr += sigma[t][v];
				}
			}

			ObjCtr = IloRange(env, expr, BestUpper, "ObjCtr");
			MasterModel.add(ObjCtr);
			expr.end();
		}
		if(Inst.AddObjLower==1){
			IloExpr expr(env);
			if(Inst.NoObj==0){
				for (int i = 0; i < Inst.Np; ++i) {
					for (int k = 0; k < Inst.Nk; ++k) {
						if(Inst.stocks[i][k]>0){
							for (int j = 0; j < Inst.Nc; ++j) {	
								if(Inst.demands[j][k]>0){
									if(Inst.FReal==0)
										expr += Inst.dist[i][j+Inst.Np+Inst.Nh] * f[i][k][j];
									else
										expr += Inst.dist[i][j+Inst.Np+Inst.Nh] * fr[i][k][j];
								}
							}
						}
					}
				}
			}
			for (int t = 0; t < Inst.Nt; ++t) {
				for (int v = 0; v < Inst.Nv; ++v) {
					expr += sigma[t][v];
				}
			}

			ObjCtrLow = IloRange(env, 0,expr, IloInfinity, "ObjCtrLow");
			MasterModel.add(ObjCtrLow);
			expr.end();
		}

		if(Inst.SigmaCuts>=1){
			AddSigmmaCuts(env,MasterCplex,WorkerCplex,WorkerModel,Inst,0,w,y,yr,sigma,u,x,AddCuts);
			MasterModel.add(AddCuts);
		}
		//MasterCplex.exportModel("filemas0.lp");	
		bool BestUpperChg=false;
		bool BestLowerChg=false;
		// Loop through each Benders iteration
		while(iter < maxIterations && !GetOut){
			BestUpperChg=false;
			auto startMaster = std::chrono::high_resolution_clock::now();
			//cout<<Inst.GAPlist[Inst.CurrGAP]<<" "<<sameLB<<endl;
			if(Inst.GAP0==1){
				MasterCplex.setParam(IloCplex::Param::TimeLimit, min(3.0,max(Inst.TimeLimit-Inst.SubSolving.count()-Inst.MasterSolving.count(),10.0)));
				MasterCplex.solve();
				if (MasterCplex.getStatus() != IloAlgorithm::Optimal && MasterCplex.getStatus()  != IloAlgorithm::Infeasible){
					Inst.GAP0=0;
					Inst.CurrGAP=0;
					MasterCplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, Inst.GAPlist[Inst.CurrGAP]);
				}
			}
			if(Inst.GAP0==0){
				MasterCplex.setParam(IloCplex::Param::TimeLimit, max(10.0,Inst.TimeLimit-Inst.SubSolving.count()-Inst.MasterSolving.count()));
				MasterCplex.solve();
			}
			auto endMaster = std::chrono::high_resolution_clock::now();
			
			Inst.MasterSolving+=endMaster-startMaster;
			AddCuts.clear();
			double value;
			if (MasterCplex.getStatus() == IloAlgorithm::Optimal){
				//cout<<MasterCplex.getObjValue()<<" "<<MasterCplex.getBestObjValue()<<endl;
				
				// Retrieve variable values for each solution in the pool
				int numSolutions = MasterCplex.getSolnPoolNsolns();
				/*std::cout << "Number of solutions found: " << numSolutions << std::endl;

				// Loop through each solution and print the objective value
				for (int i = 0; i < numSolutions; i++) {
					std::cout << "Solution " << i+1 << " has objective value: " 
							<< MasterCplex.getObjValue(i) << std::endl;
				}*/

				IloNumArray solution(env);
				//assert(lower <= MasterCplex.getObjValue() + 0.1);
				if(ceill(MasterCplex.getBestObjValue()-0.00000001)> lower){
					sameLB=0;
					lower=ceill(MasterCplex.getBestObjValue()-0.00000001);
					BestLowerChg=true;
				}else
					sameLB++;					
				IloNumVarArray varsMaster(env);
				int tempupper;    
				if(Inst.MoreSol>1){
					upper=10000;
					int sol=0;
					while ( sol < numSolutions && Inst.SubSolving.count()+Inst.MasterSolving.count()<Inst.TimeLimit) {
						tempupper=0;
						auto startMaster = std::chrono::high_resolution_clock::now();
						for (int i = 0; i < Inst.Np; i++){
							for (int j = 0; j < Inst.Nc; j++){
								for (int k = 0; k < Inst.Nk; k++){
									if(Inst.stocks[i][k]>0 && Inst.demands[j][k]>0){
										if(Inst.FReal==0)
											value=MasterCplex.getValue(f[i][k][j],sol);
										else
											value=MasterCplex.getValue(fr[i][k][j],sol);
										if(value>1-epsi){
											if(Inst.NoObj==0)
												tempupper+=Inst.dist[i][j+Inst.Np+Inst.Nh]*Inst.ImortanceObj;
										}
									}
								}
							}
						}
						tempupper+=FindCuts(env,MasterCplex,WorkerCplex,WorkerModel,Inst,sol,w,y,yr,sigma,u,x,AddCuts);
						if(tempupper < BestUpper){
							BestUpperChg=true;
							BestUpper=tempupper;
						}
						if(tempupper < upper)
							upper=tempupper;
						if(Inst.FeasFirst==1 && MasterCplex.getObjValue()== MasterCplex.getObjValue(sol) && tempupper<10000)
							Inst.FeasFirst=0;
						if(Inst.FeasFirst==2 && MasterCplex.getObjValue()== MasterCplex.getObjValue(sol) && tempupper<10000)
							Inst.FeasFirst=-1;	
						if(Inst.FeasFirst==-1 && MasterCplex.getObjValue()== MasterCplex.getObjValue(sol) && tempupper>=10000)
							Inst.FeasFirst=2;	
						auto endMaster = std::chrono::high_resolution_clock::now();
						Inst.SubSolving+=endMaster-startMaster;
						sol++;
					}
				}else{
					upper=0;
					auto startMaster = std::chrono::high_resolution_clock::now();
					for (int i = 0; i < Inst.Np; i++){
						for (int j = 0; j < Inst.Nc; j++){
							for (int k = 0; k < Inst.Nk; k++){
								if(Inst.stocks[i][k]>0 && Inst.demands[j][k]>0){
									if(Inst.FReal==0)
										value=MasterCplex.getValue(f[i][k][j]);
									else
										value=MasterCplex.getValue(fr[i][k][j]);
									if(value>1-epsi){
										if(Inst.NoObj==0)
											upper+=Inst.dist[i][j+Inst.Np+Inst.Nh]*Inst.ImortanceObj;
										if(Inst.WarmStart==1){
											if(Inst.FReal==0)
												varsMaster.add(f[i][k][j]);
											else
												varsMaster.add(fr[i][k][j]);
											}
									}
								}
							}
						}
					}
					upper+=FindCuts(env,MasterCplex,WorkerCplex,WorkerModel,Inst,-1,w,y,yr,sigma,u,x,AddCuts);
					if(upper < BestUpper){
						BestUpperChg=true;
						BestUpper=upper;
					}
					auto endMaster = std::chrono::high_resolution_clock::now();
					Inst.SubSolving+=endMaster-startMaster;
					if(Inst.FeasFirst==1 && upper<10000)
						Inst.FeasFirst=0;
					if(Inst.FeasFirst==2 && upper<10000)
						Inst.FeasFirst=-1;
					if(Inst.FeasFirst==-1 && upper>=10000)
						Inst.FeasFirst=2;
				}
				
				if(Inst.WarmStart==1)
					MasterCplex.getValues(solution, varsMaster);
				if(Inst.SubSolving.count()+Inst.MasterSolving.count()>Inst.TimeLimit){
					GetOut=true;
					cout<<"Time limit"<<endl;
					/*for (int i = 0; i < Inst.Np; i++){
						for (int j = 0; j < Inst.Nc; j++){
							for (int k = 0; k < Inst.Nk; k++){
								if(Inst.stocks[i][k]>0 && Inst.demands[j][k]>0){
									if(MasterCplex.getValue(f[i][k][j])){
										cout<<" "<<f[i][k][j];
									}
								}
							}
						}
					}*/
				}
				
				if(Inst.WarmStart==1)
					MasterCplex.addMIPStart(varsMaster, solution);
			}else if (MasterCplex.getStatus()  == IloAlgorithm::Infeasible) {
				cout<<"Problem is unfeasible"<<endl;
				//MasterCplex.exportModel("fileunf.lp");
				GetOut=true;
			}else{
				GetOut=true;
				std::cerr << "Master Solving failed" << std::endl;
			}
			if(Inst.TimeCode==1){
				ofstream FichierTime(Inst.intputFile, ios::app);
				FichierTime << iter <<" ; "<<Inst.MasterSolving.count()<<" ; "<<Inst.SubSolving.count()<<" ; "<<Inst.NbFeasCut+Inst.NbOptCut<<" ; "<<upper<<" ; "<<lower<<endl;
			}
			cout<<iter<<" "<<upper<<" "<<lower<< " NbFeas "<<Inst.NbFeasCut<<" NbOpt "<<Inst.NbOptCut<<" "<<"MasterS "<<Inst.MasterSolving.count()<<" Inst.SubSolving "<<Inst.SubSolving.count()<<" "<<(upper - lower) / upper<<" "<<Inst.GAPlist[Inst.CurrGAP]<<endl;
			/*if(Inst.nbOccuHub>0)
				cout<<"Average Hub occupation "<<Inst.OccuHub/Inst.nbOccuHub<<" max "<<Inst.MaxOccuHub<<endl;
			if(Inst.nbOccuProd>0)
				cout<<"Average Prod occupation "<<Inst.OccuProd/Inst.nbOccuProd<<" max "<<Inst.MaxOccuProd<<endl;
			if(Inst.nbLengthTourHub>0)
				cout<<"Average Hub Length Tour "<<Inst.LengthTourHub/Inst.nbLengthTourHub<<endl;
			if(Inst.nbLengthTourProd>0)
				cout<<"Average Prod Length Tour "<<Inst.LengthTourProd/Inst.nbLengthTourProd<<endl;
			Inst.MaxOccuHub=0;
			Inst.MaxOccuProd=0;
			Inst.OccuHub=0;
			Inst.OccuProd=0;
			Inst.nbOccuHub=0;
			Inst.nbOccuProd=0;
			Inst.nbLengthTourHub=0;
			Inst.nbLengthTourProd=0;
			Inst.LengthTourHub=0;
			Inst.LengthTourProd=0;*/
			
			if(iter==0)
					cout<<"Initial LB: "<<lower<<endl;
			if((upper - lower) / upper < epsi && (Inst.Gap==0 || Inst.GAPlist[Inst.CurrGAP]==0.0)){
				cout<<"Terminating with the optimal solution"<<endl;
            	cout<<"Optimal value: "<<lower<<endl;
				if(Inst.NoObj==1){
					float newopt=lower;
					for (int i = 0; i < Inst.Np; i++){
						for (int j = 0; j < Inst.Nc; j++){
							for (int k = 0; k < Inst.Nk; k++){
								if(Inst.stocks[i][k]>0 && Inst.demands[j][k]>0){
									if(Inst.FReal==0)
										value=MasterCplex.getValue(f[i][k][j]);
									else
										value=MasterCplex.getValue(fr[i][k][j]);
									if(value>1-epsi){
										newopt+=Inst.dist[i][j+Inst.Np+Inst.Nh];
									}
								}
							}
						}
					}
					cout<<"True Optimal: "<<newopt<<endl;
				}
				vector<int> Operation_Period(Inst.Nt,0);
				vector<int> Operation_Prod(Inst.Np,0);
				vector<int> Operation_Hub(Inst.Nh,0);
				for (int c = 0; c < Inst.Nc; c++){
					for(int k=0; k < Inst.Nk; k++){
						if(Inst.demands[c][k]>0){
							for(int v=0; v < Inst.Nv; v++){
								for(int t=0; t <Inst.Nt; t++){
									value = MasterCplex.getValue(w[c+Inst.Np+Inst.Nh][t][v][k][c]);
									if(value>1-epsi){
										Operation_Period[t]++;
										if(Inst.StartVehicle[v]<Inst.Np)
											Operation_Prod[Inst.StartVehicle[v]]++;
										else
											Operation_Hub[Inst.StartVehicle[v]-Inst.Np]++;
									}
								}
							}
						}
					}
				}
				int UseP=0,UseH=0,UseT=0,TotDelProd=0,TotDelHub=0;
				
				for(int t=0; t <Inst.Nt; t++){
					if(Operation_Period[t]>0){
						UseT++;
					}
				}
				for (int i = 0; i < Inst.Np; i++){
					if(Operation_Prod[i]>0){
						UseP++;
						TotDelProd+=Operation_Prod[i];
					}
				}
				for (int i = 0; i < Inst.Nh; i++){
					if(Operation_Hub[i]>0){
						UseH++;
						TotDelHub+=Operation_Hub[i];
					}
				}
				cout<<"Useful Prod "<<UseP<<endl;
				cout<<"Useful Hub "<<UseH<<endl;
				cout<<"Useful Period "<<UseT<<endl;
				cout<<"Delivery Prod "<<TotDelProd<<endl;
				cout<<"Delivery Hub "<<TotDelHub<<endl;
				GetOut=true;
			}if(Inst.Gap!=0 && Inst.GAP0==0 && Inst.Gap<5 && (upper - lower) / upper< Inst.GAPlist[Inst.CurrGAP]){
				Inst.CurrGAP=Inst.CurrGAP+1;
				sameLB=0;
				MasterCplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, Inst.GAPlist[Inst.CurrGAP]);
			}else if(Inst.Gap!=0 && Inst.GAP0==0 && Inst.Gap<5 && Inst.CurrGAP!=0 && (upper - lower) / upper > Inst.GAPlist[Inst.CurrGAP-1]){
				if(Inst.CurrGAP!=0){
					Inst.CurrGAP=Inst.CurrGAP-1;
					MasterCplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, Inst.GAPlist[Inst.CurrGAP]);
				}
			}else if(Inst.Gap>=5 && Inst.GAP0==0){
				if(AddCuts.getSize()==0){
					if(Inst.CurrGAP!=(int)Inst.GAPlist.size()-1){
						Inst.CurrGAP=Inst.CurrGAP+1;
						sameLB=0;
						MasterCplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, Inst.GAPlist[Inst.CurrGAP]);
					}
				}else{
					if(Inst.CurrGAP!=0){
						Inst.CurrGAP=Inst.CurrGAP-1;
						MasterCplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, Inst.GAPlist[Inst.CurrGAP]);
					}
				}
			}
			if(sameLB==Inst.SameLB){
				if(Inst.CurrGAP<(int)Inst.GAPlist.size()-1){
					Inst.CurrGAP=Inst.CurrGAP+1;
					MasterCplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, Inst.GAPlist[Inst.CurrGAP]);
					sameLB=0;
				}
			}
			MasterModel.add(AddCuts);
			//MasterCplex.exportModel("filemas0.lp");	
			if(Inst.AddConstraintObj==1 && BestUpperChg){
				MasterModel.remove(ObjCtr);
				ObjCtr.setUB(upper);
				MasterModel.add(ObjCtr);
			}
			if(Inst.AddObjLower==1 && BestLowerChg ){
				MasterModel.remove(ObjCtrLow);
				ObjCtrLow.setLB(lower);
				MasterModel.add(ObjCtrLow);
			}
			if(Inst.AddConstraintObj==2 && BestUpperChg){
				MasterCplex.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, upper);
			}
			if(Inst.AddObjLower==2 && BestLowerChg ){
				MasterCplex.setParam(IloCplex::Param::MIP::Tolerances::LowerCutoff,lower);
			}
			

			iter+=1;
      	}
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> duration = end - start;

		cout<<"Total time "<<duration.count()<<endl;
		cout<<"Iteration "<<iter<<endl;
		cout<<"Upper "<<BestUpper<<endl;
		cout<<"Lower "<<lower<<endl;
		cout<<"Gap "<<(BestUpper - lower) / BestUpper<<endl; 
		cout<<"Master Solving "<<Inst.MasterSolving.count()<<endl;
		cout<<"Sub Solving "<<Inst.SubSolving.count()<<endl;
		cout<<"NbFeas "<<Inst.NbFeasCut<<endl;
		cout<<"NbOpt "<<Inst.NbOptCut<<endl;
		cout<<"Average Node Subs "<<(float)Inst.NbNodeSubs/Inst.NbSolvedSubs<<endl;
		cout<<"Max Node Subs "<<Inst.MaxNode<<endl;
		cout<<"Min Node Subs "<<Inst.MinNode<<endl;
		
		if(Inst.Output!=""){
			std::ofstream outFile(Inst.Output, std::ios::app);
			outFile<<"Optimal cost "<<BestUpper<<endl;
			outFile.close();
		}
			
		/*cout<<"Average Hub occupation "<<Inst.OccuHub/Inst.nbOccuHub<<endl;
		cout<<"Average Prod occupation "<<Inst.OccuProd/Inst.nbOccuProd<<endl;*/
		AddCuts.end();
		// End the Cplex objects
		MasterCplex.end();
		WorkerCplex.end();

		// End the models
		MasterModel.end();
		WorkerModel.end();
		env.end();
	} catch (IloException& e) {
		std::cerr << "Error: " << e.getMessage() << std::endl;
	} catch (...) {
		std::cerr << "Unknown error." << std::endl;
	}
   return 0;

} //END MAIN
#endif

