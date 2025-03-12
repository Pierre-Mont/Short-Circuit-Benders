#include "MyInstance.hpp"
#include <cstdlib>  // For the system() function
#include <stdexcept>

#include "Parameters.h"
#include "Loader.h"

#include "Model.h"
#include "SolutionChecker.h"

#ifndef MYBAPCOD_HPP // Start include guard
#define MYBAPCOD_HPP

pair<float,vector<vector<int>>> SolveWithBapcod(BcInitialisation& Bc,vrpstw::Loader loader,int maxNbVehicles, int capacity, int MaxWork, int nbCust, vector<int> xCoord, vector<int> yCoord, vector<int> demand, vector<int> DistDepot,vector<pair<int,vector<int>>>& Resultat){
	
	loader.loadMyVRp(maxNbVehicles, capacity, MaxWork, nbCust, xCoord, yCoord, demand,DistDepot);

	vrpstw::Model model =  vrpstw::Model(Bc);
   
	BcSolution solution(model.master());
	
	solution = model.solve();
    
    vector<vector<int>> SolVec;
    
    //SolVec=solution.GetVisitedVertices();
    Resultat=solution.tt();
    
    
	model.clearModel();
    loader.ClearData();
	Bc.bcReset();
    if(solution.defined()){
        return {solution.cost(),SolVec};
    }else{
        return {-1,{}};
    }
    
}
#endif