#include <ilcplex/ilocplex.h>
#include <string>
#include "MyInstance.hpp"
#include <chrono>

ILOSTLBEGIN


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
void createMasterModel(IloEnv& env, IloModel& masterModel, MyInstance Inst, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>>& w, IloArray<IloArray<IloNumVar>>& sigma,IloArray<IloArray<IloBoolVarArray>>& f,IloArray<IloArray<IloBoolVarArray>>& y) {
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
						if(i<Inst.Np+Inst.Nh || i > Inst.Np + Inst.Nh + Inst.Nc -1 || c==i-Inst.Np-Inst.Nh ){
							ostringstream varName;
							varName << "w_" << i << "_" << t << "_" << v << "_"<<k <<"_"<<c;
							w[i][t][v][k][c] = IloBoolVar(env, varName.str().c_str());
						}
					}
				}
			}
		}
	}
	for (int i = 0; i < Inst.Np; ++i) {
		f[i] = IloArray<IloBoolVarArray>(env, Inst.Nk);
		for (int k = 0; k < Inst.Nk; ++k) {
			f[i][k] = IloBoolVarArray(env, Inst.Nc);
			if(Inst.stocks[i][k]>0){
				for (int c = 0; c < Inst.Nc; ++c) {
					if(Inst.demands[c][k]>0){
						ostringstream varName;
						varName << "f_" << i <<"_"<<k <<"_"<<c;
						f[i][k][c] = IloBoolVar(env, varName.str().c_str());
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
			sigma[t][v] = IloNumVar(env, 0.0, IloInfinity,varName.str().c_str());
		}
	}
	// Define the objective function
	IloExpr objective(env);
	for (int i = 0; i < Inst.Np; ++i) {
		for (int k = 0; k < Inst.Nk; ++k) {
			if(Inst.stocks[i][k]>0){
				for (int j = 0; j < Inst.Nc; ++j) {	
					if(Inst.demands[j][k]>0)
						objective += Inst.dist[i][j+Inst.Np+Inst.Nh] * f[i][k][j];
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
					if(Inst.stocks[i][k]>0)
						client_service += f[i][k][j];
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
				if(Inst.demands[j][k]>0)
					stock_ok += f[i][k][j] * Inst.demands[j][k];
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
					masterModel.add(command_leave == f[i][k][c]);
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
						masterModel.add(w[i][t][v][k][c] <= Inst.Prod_av[i][t]);
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
									if(Inst.StartVehicle[v]>= Inst.Np || Inst.StartVehicle[v]==i)
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
	//Prod must drop 
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
	prod_drop.end();
	//Hub must drop 
	for (int i = 0; i < Inst.Nh; ++i) {
		for (int k = 0; k < Inst.Nk; ++k) {
			for (int c = 0; c < Inst.Nc; ++c) {
				if(Inst.demands[c][k]>0){
					for (int t = 0; t < Inst.Nt; ++t) {
						for (int v : Inst.Vehicles[i + Inst.Np]) {
							IloExpr hub_drop_all(env);
							for (int j : Inst.Pplus) {
								if (Inst.PairHub[i].first != j) {	
									hub_drop_all += w[j][t][v][k][c];
								}
							}
							masterModel.add(hub_drop_all == w[Inst.PairHub[i].second][t][v][k][c]);
							hub_drop_all.end();
						}
					}
				}
			}
		}
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
	// Constraint 2: CapacityOK
	/*for (int t = 0; t < Inst.Nt; ++t) {
		for (int i = 0; i < Inst.Np; ++i) {
			for (int v: Inst.Vehicles[i]) {
				IloExpr expr(env);
				for (int k = 0; k < Inst.Nk; ++k) {
					for (int c = 0; c < Inst.Nc; ++c) {
						expr += w[i][t][v][k][c] * Inst.demands[c][k] * Inst.Psize[k];
					}
				}
				masterModel.add(expr <= Inst.CapaVehicle[v]);
				expr.end();
			}
			
		}
	}*/
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
	for (int t = 0; t < Inst.Nt; ++t) {
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
				masterModel.add(expr <= Inst.CapaVehicle[v]*Inst.TourVehicle[v]);
				expr.end();
			}
		}
	}


	// Constraint 3: ProdCantPick
	for (int t = 0; t < Inst.Nt; ++t) {
		for (int i = 0; i < Inst.Np; ++i) {
			for (int v: Inst.Vehicles[i]) {
				IloExpr expr(env);
				for (int j : Inst.Pplus) {
					if (Inst.Pplus[j] != i) {
						for (int k = 0; k < Inst.Nk; ++k) {
							for (int c = 0; c < Inst.Nc; ++c) {
								expr += w[j][t][v][k][c];
							}
						}
					}
				}
				masterModel.add(expr == 0);
				expr.end();
			}
		}
	}
	bool atleastone;
	// Constraint 4: TooFarPoint
	for (int i = 0; i < Inst.Np; ++i) {
		IloExpr expr(env);
		atleastone=false;
		for (int c = 0; c < Inst.Nc; c++){
			if(Inst.dist[i][c+Inst.Np+Inst.Nh] > (Inst.WorkVehicle[i] / 2.0)){
				for (int k = 0; k < Inst.Nk; k++){
					if(Inst.demands[c][k]>0 && Inst.stocks[i][k]>0){
						expr += f[i][k][c];
						atleastone=true;
					}
				}
			}
		}
		if(atleastone)
			masterModel.add(expr == 0);
		expr.end();
	}

	if(Inst.ImprovedCut==1){
		
		for (int i = 0; i < Inst.node; ++i) {
		y[i] = IloArray<IloBoolVarArray>(env, Inst.Nt);
			for (int t = 0; t < Inst.Nt; ++t) {
				y[i][t] = IloBoolVarArray(env, Inst.Nc);
				for (int v = 0; v < Inst.Nv; ++v) {
					ostringstream varName;
					varName << "y_" << i <<"_"<<t <<"_"<<v;
					y[i][t][v] = IloBoolVar(env, varName.str().c_str());
				}	
			}
		}

		for (int i = 0; i < Inst.node; ++i) {
			for (int t = 0; t < Inst.Nt; ++t) {
				for (int v = 0; v < Inst.Nv; ++v) {
					for (int k = 0; k < Inst.Nk; ++k) {
						for (int c = 0; c < Inst.Nc; ++c) {
							if(i<Inst.Np+Inst.Nh || i > Inst.Np + Inst.Nh + Inst.Nc -1 || c==i-Inst.Np-Inst.Nh )
								masterModel.add(w[i][t][v][k][c] <= y[i][t][v]);
						}
					}
				}
				
			}
		}

	}
	if(Inst.SigmaCuts==1){
		for (int t = 0; t < Inst.Nt; t++){
			for (int v = 0; v < Inst.Nv; v++){
				for(int i=0; i<Inst.node;i++){
					if(i!=Inst.StartVehicle[v]){
						for (int k = 0; k < Inst.Nk; k++){
							for (int c = 0; c < Inst.Nc; c++){
								if(i<Inst.Np+Inst.Nh || i > Inst.Np + Inst.Nh + Inst.Nc -1 || c==i-Inst.Np-Inst.Nh )
									masterModel.add(sigma[t][v]>= 2*Inst.dist[i][Inst.StartVehicle[v]] * w[i][t][v][k][c]);
							}
						}
					}
				}
			}
		}
		
	}
	if(Inst.CapH==1){
		for (int t = 0; t < Inst.Nt; t++){
			for (int v = 0; v < Inst.Nv; v++){
				for (int i = 0; i < Inst.Np; i++){
					IloExpr expr(env);
					for (int k = 0; k < Inst.Nk; ++k) {
						for (int c = 0; c < Inst.Nc; ++c) {
							expr += w[i][t][v][k][c] * Inst.demands[c][k] * Inst.Psize[k];
						}
					}
					masterModel.add(expr <= Inst.CapaVehicle[v]);
					expr.end();
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

   IloExpr expr(env);
   int NbNode= (int) NodeSub.size();
   for (int j = 1; j < NbNode; ++j) {   
	  expr.clear();
      for (int i = 0; i < NbNode; ++i) {
         if (NodeSub[i] != NodeSub[j]) {
            for (int r = 0; r < MaxTour; ++r) {
                  expr += x[NodeSub[i]][NodeSub[j]][r];
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
            if (NodeSub[i] != NodeSub[j]) {
                  expr += x[NodeSub[i]][NodeSub[j]][r];
                  flowdepot_in += x[NodeSub[j]][NodeSub[i]][r];
            }
         }
         workerModel.add(expr == flowdepot_in);
         flowdepot_in.end();
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

void AddFeasCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w, vector<array<int,3>> Var,vector<array<int,3>> Var2, IloConstraintArray& Addcuts){
	
	
	IloExpr expr(env);
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
	expr.end();
}

void ImprovedAddFeasCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloBoolVarArray>>& y, vector<int> Var,vector<int> Var2, IloConstraintArray& Addcuts){
	
	IloExpr expr(env);
	for (int t = 0; t < Inst.Nt; t++){
		if(currnode >= Inst.Np || Inst.Prod_av[currnode][t]){
			for (int v = 0; v < Inst.Nv; v++){
				if(Inst.StartVehicle[v]==currnode){
					expr.clear();
					for (int i = 0; i < (int)Var.size(); ++i) {
						expr += y[Var[i]][t][v];  
					}
					for (int i = 0; i < (int)Var2.size(); ++i) {
						expr += y[Var2[i]][t][v];  
					}
					Inst.NbFeasCut++;
					Addcuts.add(expr <=  (int)(Var2.size()+Var.size()-1));  
				}
			}
		}
	}
	expr.end();
}
void AddOptCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w,  IloArray<IloArray<IloNumVar>>& sigma, float Opt, vector<array<int,3>> Var, vector<array<int,3>> Var2, IloConstraintArray& Addcuts){
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
	expr.end();
}

void AddImprovedHubOptCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w, IloArray<IloArray<IloBoolVarArray>> y, IloArray<IloArray<IloNumVar>>& sigma, float Opt, vector<array<int,3>> Var, vector<array<int,3>> Var2,vector<int> VarNotIn, IloConstraintArray& Addcuts){
	IloExpr expr(env);
	float init;
	cout<<"OPT is "<<Opt<<endl;
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
	cout<<" ADD OPT CUT"<<endl;
	for (int t = 0; t < Inst.Nt; t++){
		//Add a cut only if the current node is a hub or an available producer.
		if(currnode >= Inst.Np || Inst.Prod_av[currnode][t]){
			for (int v = 0; v < Inst.Nv; v++){
				if(Inst.StartVehicle[v]==currnode){
					expr.clear();
					init=Opt;
					for (int i = 0; i < (int)Var.size(); ++i) {
						init-=2*Inst.dist[currnode][Var[i][0]];
						expr +=  w[Var[i][0]][t][v][Var[i][1]][Var[i][2]] * 2 *Inst.dist[currnode][Var[i][0]];  // Build an expression with the variables in OneVal
					}
					for (int i = 0; i < (int)Var2.size(); ++i) {
						expr += w[Var2[i][0]][t][v][Var2[i][1]][Var2[i][2]] * 2 *Inst.dist[currnode][Var2[i][0]]; // Build an expression with the variables in OneVal
						init-=2*Inst.dist[currnode][Var2[i][0]];
					}

					for (int i = 0; i < (int)VarNotIn.size(); ++i) {
						expr += y[VarNotIn[i]][t][v] *Inst.MinDist[VarNotIn[i]]; // Build an expression with the variables in OneVal
					}
					Inst.NbOptCut++;
					cout<<expr<<endl;
					Addcuts.add(init + expr <= sigma[t][v]);  // Add a constraint using OneVal
				}
			}
		}
	}
	expr.end();
}
void AddImprovedProdOptCut(IloEnv& env, MyInstance& Inst, int currnode, IloArray<IloArray<IloBoolVarArray>> y,  IloArray<IloArray<IloNumVar>> sigma, float Opt, vector<int> Var,vector<int> Var2, vector<int> VarNotIn, IloConstraintArray& Addcuts){
	IloExpr expr(env);
	float init;
	for (int t = 0; t < Inst.Nt; t++){
		//Add a cut only if the current node is a hub or an available producer.
		if(currnode >= Inst.Np || Inst.Prod_av[currnode][t]){
			for (int v = 0; v < Inst.Nv; v++){
				if(Inst.StartVehicle[v]==currnode){
					expr.clear();
					init=Opt;
					for (int i = 0; i < (int)Var.size(); ++i) {
						init-=2*Inst.dist[currnode][Var[i]];
						expr +=  y[Var[i]][t][v] * Inst.dist[currnode][Var[i]];  // Build an expression with the variables in OneVal
					}
					for (int i = 0; i < (int)Var2.size(); ++i) {
						expr += y[Var2[i]][t][v] *Inst.dist[currnode][Var2[i]]; // Build an expression with the variables in OneVal
						init-=2*Inst.dist[currnode][Var2[i]];
					}

					Inst.NbOptCut++;
					Addcuts.add(init + 2*expr <= sigma[t][v]);  // Add a constraint using OneVal
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
						AddFeasCut(env,Inst,currnode,w,tempvec,VarDeli,Addcuts);
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
					AddFeasCut(env,Inst,currnode,w,VarPick,tempvec,Addcuts);
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
						AddOptCut(env,Inst,currnode,w,sigma,Opt,tempvec,VarDeli,Addcuts);
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
					AddOptCut(env,Inst,currnode,w,sigma,Opt,VarPick,tempvec,Addcuts);
					tempvec[i]={VarDeli[i][0],VarDeli[i][1],VarDeli[i][2]};
				}
			}
		}
	}
}
int mainBend(MyInstance Inst)
{
	try {
		
		cout << "Model the problem" << std::endl;
		IloEnv env;	
		IloModel MasterModel(env);
		IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> w(env, Inst.node);
		IloArray<IloArray<IloNumVar>> sigma(env, Inst.Nt);
		IloArray<IloArray<IloBoolVarArray>> f(env, Inst.Np);
		IloArray<IloArray<IloBoolVarArray>> y(env, Inst.node);
		createMasterModel(env,MasterModel,Inst,w,sigma,f,y);
		IloModel WorkerModel(env);
		IloArray<IloArray<IloNumVar>> u(env, Inst.node);
		IloArray<IloArray<IloArray<IloBoolVar>>> x(env, Inst.node);
		createWorkerModel(env,WorkerModel,Inst,u,x);
		
		IloCplex MasterCplex(MasterModel);
		IloCplex WorkerCplex(WorkerModel);
		
		MasterCplex.setOut(env.getNullStream());  // Suppress log output
		MasterCplex.setWarning(env.getNullStream());
		MasterCplex.setParam(IloCplex::Param::MIP::Display, 0);
		MasterCplex.setParam(IloCplex::IloCplex::Param::MIP::Tolerances::Integrality, 1e-9);  // Integer feasibility tolerance
		MasterCplex.setParam(IloCplex::Param::Simplex::Tolerances::Optimality, 1e-9);  // Optimality tolerance
		MasterCplex.setParam(IloCplex::Param::Feasopt::Tolerance, 1e-9);
		MasterCplex.setParam(IloCplex::Param::Threads, 1);

		WorkerCplex.setParam(IloCplex::Param::Threads, 1);
		WorkerCplex.setParam(IloCplex::Param::MIP::Display, 0);
		WorkerCplex.setOut(env.getNullStream());  // Suppress log output
		WorkerCplex.setParam(IloCplex::IloCplex::Param::MIP::Tolerances::Integrality, 1e-9);  // Integer feasibility tolerance
		WorkerCplex.setParam(IloCplex::Param::Simplex::Tolerances::Optimality, 1e-9);  // Optimality tolerance
		WorkerCplex.setParam(IloCplex::Param::Feasopt::Tolerance, 1e-9);
		WorkerCplex.setWarning(env.getNullStream());
		int maxIterations=1000,iter=0;
		pair<int,int> TotalDem;
		bool AlreadyAdded, GetOut=false;
		vector<int> NodeSub,dem, NotUsedPoint;
		pair<vector<int>,vector<int>> PickAndDel,DemandPickAndDel(vector<int>(Inst.node,0),vector<int>(Inst.node,0));
		IloConstraintArray AddCuts(env);
		float lower=0,BestUpper=10000,upper=0,epsi = 1e-5,tot;
		vector<array<int,3>> VarPick, VarDeli,empt;
    	MasterCplex.exportModel("filemas.lp");	
		auto start = std::chrono::high_resolution_clock::now();

		
		std::chrono::duration<double> MasterSolving(0.0),SubSolving(0.0);
		cout<<"Solve the Problem"<<endl;
		// Loop through each Benders iteration
		while(iter < maxIterations && !GetOut){
			// Solve master problem
			auto startMaster = std::chrono::high_resolution_clock::now();
			MasterCplex.solve();
			auto endMaster = std::chrono::high_resolution_clock::now();

			MasterSolving+=endMaster-startMaster;
			AddCuts.clear();
			//MasterCplex.exportModel("filemas2.lp");	
			
			double value;
			if (MasterCplex.getStatus() == IloAlgorithm::Optimal){
				//cout<<lower<< " "<<MasterCplex.getObjValue()<<endl;

				assert(lower <= MasterCplex.getObjValue() + 0.1);
				lower=MasterCplex.getObjValue(); 
				IloNumArray solution(env);
				IloNumVarArray varsMaster(env);       
					
				upper=0;
				for (int i = 0; i < Inst.Np; i++){
					for (int j = 0; j < Inst.Nc; j++){
						for (int k = 0; k < Inst.Nk; k++){
							if(Inst.stocks[i][k]>0 && Inst.demands[j][k]>0){
								value=MasterCplex.getValue(f[i][k][j]);
								if(value>1-epsi){
									upper+=Inst.dist[i][j+Inst.Np+Inst.Nh];
									//cout<<" "<<f[i][k][j];
									if(Inst.WarmStart==1)
										varsMaster.add(f[i][k][j]);
									}
							}
						}
					}
				}
				//cout<<"init "<<upper<<endl;
				//cout<<endl;
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
							NotUsedPoint.clear();
							tot=0;
							TotalDem.first=0;
							TotalDem.second=0;

							/*for (int i = 0; i < Inst.node; ++i) {
									AlreadyAdded=false;
									for (int k = 0; k < Inst.Nk; ++k) {
										for (int c = 0; c < Inst.Nc; ++c) {
											if(Inst.demands[c][k]>0){
												if(i<Inst.Np+Inst.Nh || i > Inst.Np + Inst.Nh + Inst.Nc -1 || c==i-Inst.Np-Inst.Nh ){
													value = MasterCplex.getValue(w[i][t][v][k][c]);
													if(value>1-epsi){
														if(i<Inst.Np + Inst.Nh){
															cout<<"pick "<<w[i][t][v][k][c]<< " "<<Inst.demands[c][k]*Inst.Psize[k]<<" " <<Inst.dist[i][Inst.StartVehicle[v]]<<endl;
													
														}else{
															cout<<"depot "<<w[i][t][v][k][c]<<" "<<Inst.demands[c][k]*Inst.Psize[k]<<" "<<Inst.dist[i][Inst.StartVehicle[v]]<<endl;
															
														}
													}
												}
											}
										}
									
									
								}
							}*/
							

							for (int i = 0; i < Inst.node; ++i) {
								if(i!=Inst.StartVehicle[v] && (Inst.StartVehicle[v]<Inst.Np || Inst.PairHub[Inst.StartVehicle[v]-Inst.Np].second !=i )){
									AlreadyAdded=false;
									for (int k = 0; k < Inst.Nk; ++k) {
										for (int c = 0; c < Inst.Nc; ++c) {
											if(Inst.demands[c][k]>0){
												if(i<Inst.Np+Inst.Nh || i > Inst.Np + Inst.Nh + Inst.Nc -1 || c==i-Inst.Np-Inst.Nh ){
													value = MasterCplex.getValue(w[i][t][v][k][c]);
													if(value>1-epsi){
														if(Inst.WarmStart==1)
															varsMaster.add(w[i][t][v][k][c]);
														if(i<Inst.Np + Inst.Nh){
															//cout<<"pick "<<w[i][t][v][k][c]<<endl;
															if(!AlreadyAdded){
																AlreadyAdded=true;
																PickAndDel.first.push_back(i);
															}
															DemandPickAndDel.first[i]+=Inst.demands[c][k]*Inst.Psize[k];
															//cout<<Inst.demands[c][k]<<" "<<Inst.Psize[k]<<endl;
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
									if(!AlreadyAdded)
										NotUsedPoint.push_back(i);
								}
							}	

							vector<float> res;
							res.push_back(0);
							res.push_back(0);
							bool UNF=false;
							for (int s = 0; s < 2; s++){
								if(s==1){
									NodeSub=PickAndDel.first;
									dem=DemandPickAndDel.first;
								}
								else{
									NodeSub=PickAndDel.second;
									dem=DemandPickAndDel.second;
								}
								if(NodeSub.size()>1){
									Inst.NbNodeSubs+=NodeSub.size();
									Inst.NbSolvedSubs++;
									if((int)NodeSub.size()> Inst.MaxNode)
										Inst.MaxNode=NodeSub.size();
									if((int)NodeSub.size()<Inst.MinNode)
										Inst.MinNode=NodeSub.size();
									IloConstraintArray cons(WorkerModel.getEnv());
									for (IloModel::Iterator it(WorkerModel); it.ok(); ++it) {
										if ( (*it).isConstraint() )
											cons.add((*it).asConstraint());
									}
									WorkerModel.remove(cons);
									cons.endElements();
									cons.end();
									//cout<<"t "<<t <<" v "<<v<<" sig "<< MasterCplex.getValue(sigma[t][v])<<endl;
									//cout<<NodeSub<<endl;
									GenWorkerModel(env, WorkerModel,Inst,u,x,NodeSub,dem, Inst.TourVehicle[v],Inst.WorkVehicle[v],Inst.CapaVehicle[v]);
									//WorkerCplex.exportModel("file.sav");
									
									auto startMaster = std::chrono::high_resolution_clock::now();
									WorkerCplex.solve();
									auto endMaster = std::chrono::high_resolution_clock::now();
									SubSolving+=endMaster-startMaster;
									IloAlgorithm::Status status = WorkerCplex.getStatus();  // Get the termination status
									if (status == IloAlgorithm::Optimal) {  // If the status is optimal
										res[s]=WorkerCplex.getObjValue();         // Return the objective value
										upper+=res[s];
										tot+=res[s];
										/*cout<<"Real Cost "<<res[s]<<endl;
										//WorkerCplex.exportModel("filework.lp");
										for (int i = 0; i < (int) NodeSub.size(); i++)
										{
											for (int j = 0; j < (int) NodeSub.size(); j++){
												if (NodeSub[i] != NodeSub[j]) {
													for (int r = 0; r < Inst.TourVehicle[v]; ++r) {
														value=WorkerCplex.getValue(x[NodeSub[i]][NodeSub[j]][r]);
														if(value >1-epsi)
															cout<<x[NodeSub[i]][NodeSub[j]][r]<<" "<<Inst.dist[NodeSub[i]][NodeSub[j]]<<endl;
													}
												}
											}
										}*/
									} 
									else if (status == IloAlgorithm::Infeasible) {  // If the status is infeasible
										res[s]=-1;                                  // Return -1
										//cout<<"SUB PROBLEM IS UNFEASIBLE"<<endl;
										upper+=1000;
										UNF=true;
									} 
									else {  // For all other cases
										std::cerr << "Subproblem Solving failed" << std::endl;
										res[s]=-2;  // Optionally return -1 to indicate failure
									}
									if(res[s]==-1){
										if(s==1){
											if(Inst.ImprovedCut==0 || TotalDem.first>Inst.CapaVehicle[v]){
												AddFeasCut(env,Inst,Inst.StartVehicle[v],w,VarPick,{},AddCuts);
											}
											else
												ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,PickAndDel.first,{},AddCuts);
											if(Inst.MoreCuts==1)
													AddMoreFeasCut(env,Inst,Inst.StartVehicle[v],w,VarPick,{},AddCuts);
										}else{
											if(Inst.ImprovedCut==0 || TotalDem.second > Inst.CapaVehicle[v]){
												AddFeasCut(env,Inst,Inst.StartVehicle[v],w,VarDeli,{},AddCuts);
											}else
												ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,PickAndDel.second,{},AddCuts);
											if(Inst.MoreCuts==1)
													AddMoreFeasCut(env,Inst,Inst.StartVehicle[v],w,{},VarDeli,AddCuts);
										}
										res[s]=0;
									}else if(Inst.PartialCut==1 && res[s]>0){
										if(s==1){
											if(res[0]>0){
												if(Inst.ImprovedCut==0 || TotalDem.first>Inst.CapaVehicle[v])
													AddOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[s],VarPick,{},AddCuts);
												else
													AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,res[s],PickAndDel.first,{},NotUsedPoint,AddCuts);
												if(Inst.MoreCuts==1)
													AddMoreOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[s],VarPick,{},AddCuts);
											}
										}else{
											if(PickAndDel.second.size()>1){
												if(Inst.ImprovedCut==0 || TotalDem.second>Inst.CapaVehicle[v])
													AddOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[s],VarDeli,{},AddCuts);
												else
													AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,res[s],PickAndDel.second,{},NotUsedPoint,AddCuts);
												if(Inst.MoreCuts==1)
													AddMoreOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,res[s],{},VarDeli,AddCuts);
											}
										}
									}
								}
							}
							if(tot>epsi){
								//cout<<MasterCplex.getValue(sigma[t][v]) <<" "<<tot<<endl;
								assert(MasterCplex.getValue(sigma[t][v]) <= tot+ 0.1 || UNF);
								if(tot>Inst.WorkVehicle[v]){
									//cout<<"tot "<<tot<<" "<<Inst.WorkVehicle[v]<<endl;
									if(Inst.ImprovedCut==0 || TotalDem.first>Inst.CapaVehicle[v] || TotalDem.second > Inst.CapaVehicle[v]){
										AddFeasCut(env,Inst,Inst.StartVehicle[v],w,VarPick,VarDeli,AddCuts);
									}else
										ImprovedAddFeasCut(env,Inst,Inst.StartVehicle[v],y,PickAndDel.first,PickAndDel.second,AddCuts);
									if(Inst.MoreCuts==1)
											AddMoreFeasCut(env,Inst,Inst.StartVehicle[v],w,VarPick,VarDeli,AddCuts);											
									upper+=1000;
									if(Inst.ImprovedCut==0 || TotalDem.first>Inst.CapaVehicle[v] || TotalDem.second > Inst.CapaVehicle[v])
										AddOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,tot,VarPick,VarDeli,AddCuts);
									else
										AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,tot,PickAndDel.first,PickAndDel.second,NotUsedPoint,AddCuts);
									if(Inst.MoreCuts==1)
										AddMoreOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,tot,VarPick,VarDeli,AddCuts);
								}else if(tot>MasterCplex.getValue(sigma[t][v])+epsi){
									if(Inst.ImprovedCut==0 || TotalDem.first>Inst.CapaVehicle[v] || TotalDem.second > Inst.CapaVehicle[v])
										AddOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,tot,VarPick,VarDeli,AddCuts);
									else
										AddImprovedProdOptCut(env,Inst,Inst.StartVehicle[v],y,sigma,tot,PickAndDel.first,PickAndDel.second,NotUsedPoint,AddCuts);
									if(Inst.MoreCuts==1)
										AddMoreOptCut(env,Inst,Inst.StartVehicle[v],w,sigma,tot,VarPick,VarDeli,AddCuts);						
								}
							}
						}		  
					}
				}
				if(Inst.WarmStart==1)
					MasterCplex.getValues(solution, varsMaster);
				if(SubSolving.count()+MasterSolving.count()>1800){
					GetOut=true;
					cout<<"Time limit"<<endl;
					for (int i = 0; i < Inst.Np; i++){
						for (int j = 0; j < Inst.Nc; j++){
							for (int k = 0; k < Inst.Nk; k++){
								if(Inst.stocks[i][k]>0 && Inst.demands[j][k]>0){
									if(MasterCplex.getValue(f[i][k][j])){
										cout<<" "<<f[i][k][j];
										}
								}
							}
						}
					}
				}
				if(upper < BestUpper)
					BestUpper=upper;
				if(Inst.WarmStart==1)
					MasterCplex.addMIPStart(varsMaster, solution);
			}else if (MasterCplex.getStatus()  == IloAlgorithm::Infeasible) {
				cout<<"Problem is unfeasible"<<endl;
				GetOut=true;
			}else{
				GetOut=true;
				std::cerr << "Master Solving failed" << std::endl;
			}
			cout<<iter<<" "<<upper<<" "<<lower<< " NbFeas "<<Inst.NbFeasCut<<" NbOpt "<<Inst.NbOptCut<<" "<<"MasterS "<<MasterSolving.count()<<" SubSolving "<<SubSolving.count()<<endl;
			if((upper - lower) / upper < epsi){
				cout<<"Terminating with the optimal solution"<<endl;
            	cout<<"Optimal value: "<<lower<<endl;
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
			}
			
			MasterModel.add(AddCuts);
			iter+=1;
      	}
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> duration = end - start;
		MasterCplex.exportModel("filemas.lp");	

		cout<<"Total time "<<duration.count()<<endl;
		cout<<"Iteration "<<iter<<endl;
		cout<<"Upper "<<BestUpper<<endl;
		cout<<"Lower "<<lower<<endl;
		cout<<"Master Solving "<<MasterSolving.count()<<endl;
		cout<<"Sub Solving "<<SubSolving.count()<<endl;
		cout<<"NbFeas "<<Inst.NbFeasCut<<endl;
		cout<<"NbOpt "<<Inst.NbOptCut<<endl;
		cout<<"Average Node Subs "<<(float)Inst.NbNodeSubs/Inst.NbSolvedSubs<<endl;
		cout<<"Max Node Subs "<<Inst.MaxNode<<endl;
		cout<<"Min Node Subs "<<Inst.MinNode<<endl;
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


