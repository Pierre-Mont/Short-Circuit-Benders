#include "MyInstance.hpp"
#include <cstdlib>  // For the system() function
#include <stdexcept>

#include "Parameters.h"
#include "Loader.h"

#include "Model.h"
#include "SolutionChecker.h"



/*int SolveWithBapcod(MyBapcod bap,int maxNbVehicles, int capacity, int MaxWork, int nbCust, vector<int> xCoord, vector<int> yCoord, vector<int> demand)
{
    cout<<"Try solve with bapcod"<<endl;
	bap.loader.MyloadData(maxNbVehicles, capacity, MaxWork, nbCust, xCoord, yCoord, demand);


	BcSolution solution(model.master());
	solution = model.solve();
	
	sol_checker->isFeasible(solution, true, true);
    
	return 0;
}

void SolveWithBapcod(int maxNbVehicles, int capacity, int MaxWork, int nbCust, vector<int> xCoord, vector<int> yCoord, vector<int> demand)
{

    std::string strArray[] = {"VRP","-b ","/home/kisouke/Short-Circuit-Benders/conf/bc.cfg","-a","/home/kisouke/Short-Circuit-Benders/BapCod_conf/app.cfg","-i","/home/kisouke/bapcod-v0.82.5/Applications/VRPBenders/data/VRPTW/C101.txt"};
    const int numStrings = 7;

    // Allocate `char**`
    char** cStrings = new char*[numStrings];

    // Convert std::string to C-style string (char*) and store in `cStrings`
    for (int i = 0; i < numStrings; ++i) {
        cStrings[i] = new char[strArray[i].length() + 1];  // +1 for null terminator
        strcpy(cStrings[i], strArray[i].c_str());         // Copy the std::string to char*
    }
    cout<<"begin bapcode"<<endl;

    cout<<xCoord<<" "<<yCoord<<" "<<demand<<endl;
    BcInitialisation bapcodInit;
    vrpstw::Loader loader;
    loader.loadParameters("/home/kisouke/Short-Circuit-Benders/conf/bc.cfg",numStrings,cStrings);
    cout<<maxNbVehicles<<", "<<capacity<<", "<<MaxWork<<", "<<nbCust<<", "<<xCoord<<", "<<yCoord<<", "<<demand<<endl;
    loader.MyloadData(maxNbVehicles, capacity, MaxWork, nbCust, xCoord, yCoord, demand);

    vrpstw::Model model(bapcodInit);
    vrpstw::SolutionChecker * sol_checker=new vrpstw::SolutionChecker;
    model.attach(sol_checker);
	BcSolution solution(model.master());
	solution = model.solve();
	
	sol_checker->isFeasible(solution, true, true);
    cout<<"palepale "<<endl;
    bapcodInit.bcReset();
    cout<<"end bapcode"<<endl;
    cout<<"a^p^ldq "<<endl;
}*/


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





/*std::string exec(const char* cmd) {
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
float SolveWithBapcod(int maxNbVehicles, int capacity, int MaxWork, int nbCust, vector<int> xCoord, vector<int> yCoord, vector<int> demand)
{
    std::ofstream outfile("VRPTW/Bap.txt");
    
    // Check if the file opened successfully
    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
    }

    outfile << "C201\n\n";
    outfile << "VEHICLE\n";
    outfile << "NUMBER     CAPACITY\n";
    outfile << std::setw(5) << maxNbVehicles << std::setw(11) << capacity << "\n\n";
    outfile << "CUSTOMER\n";
    outfile << "CUST NO.  XCOORD.    YCOORD.    DEMAND   READY TIME  DUE DATE   SERVICE TIME\n\n";

    // Loop over customers and write their data
    for (int i = 0; i < nbCust; ++i) {
        outfile << std::setw(5) << i << std::setw(10) << xCoord[i] << std::setw(11) << yCoord[i]
                << std::setw(11) << demand[i] << std::setw(13) << 0 << std::setw(10) << 10000
                << std::setw(13) << 0 << "\n";
    }

    // Close the file
    outfile.close();
    string command="/home/kisouke/bapcod-v0.82.5/Demos/VehicleRoutingWithTimeWindowsDemo/bin/VehicleRoutingWithTimeWindowsDemo -b /home/kisouke/bapcod-v0.82.5/Demos/VehicleRoutingWithTimeWindowsDemo/config/bc.cfg -a /home/kisouke/bapcod-v0.82.5/Demos/VehicleRoutingWithTimeWindowsDemo/config/app.cfg -i VRPTW/Bap.txt | awk 'BEGIN { statut = 0; opt = 0 } "
                          "/infeasibility/ { statut = -1; opt = -1 } "
                          "/Best found solution of value [0-9]+/ { statut = 1; match($0, /value ([0-9]+\\.[0-9]+)/, arr); opt = arr[1] } "
                          "END { print \"\" statut \" \" opt }'";
    
    string output = exec(command.c_str());
    int statut = 0;
    float opt = 0.0f;
    
    sscanf(output.c_str(), "%d %f", &statut, &opt);  // Scan output into C++ variables
    
    if(statut!=1){
        assert(opt==-1);
    }
    return opt;
}*/