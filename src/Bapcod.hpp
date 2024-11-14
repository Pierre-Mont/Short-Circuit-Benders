#include "MyInstance.hpp"
#include <cstdlib>  // For the system() function
#include <stdexcept>

#include "Parameters.h"
#include "Loader.h"

#include "Model.h"
#include "SolutionChecker.h"

#ifndef MYBAPCOD_HPP // Start include guard
#define MYBAPCOD_HPP

float SolveWithBapcod(BcInitialisation& Bc,vrpstw::Loader loader,int maxNbVehicles, int capacity, int MaxWork, int nbCust, vector<int> xCoord, vector<int> yCoord, vector<int> demand){
	
	loader.loadMyVRp(maxNbVehicles, capacity, MaxWork, nbCust, xCoord, yCoord, demand);

	vrpstw::Model model =  vrpstw::Model(Bc);
   /* vrpstw::SolutionChecker * sol_checker = new vrpstw::SolutionChecker;
	model.attach(sol_checker);*/
	BcSolution solution(model.master());
	
	solution = model.solve();
	
    
	model.clearModel();
    loader.ClearData();
	Bc.bcReset();
    if(solution.defined()){
        return solution.cost();
    }else{
        return -1;
    }
    
}
#endif