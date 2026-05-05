#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <limits>
#include <unistd.h>
#include <cassert>
#include "Bender.hpp"
#include "Heuristic.hpp"
#include "Cplex.hpp"

static void printUsage(const char *prog) {
	std::cout
	    << "Usage: " << prog << " <instance.data> [options]\n\n"
	    << "The instance file must be the first argument.\n\n"
	    << "Options (NAME=value, all integer unless noted):\n"
		<< "Most options are heuristics options, with several versions. Here is the recommanded usage: \n"
		<< "-FR=1 -BC=2 -IC=2 -GAP=2 -MS=20 -H=2 -YT=2 -FF=1 -TL=1800 \n"
		<< "To solve the problem using CPLEX use \n <-H=2 -CPL=1> \n"
		<< "Here are the others options: \n"
	    << "  -h, --help   Show this message and exit.\n"
	    << "  -IC=[0,1,2]  Enable Improved Feasibility Cut \n   -MC=[0,1]   Attemp to generate more cuts      \n -SC=[0,1,2,3] Generate initial cuts on sigma variables\n"
	    << "  -WS=[0,1]   WarmStart for master problem\n"
	    << "  -BC=[0,1,2]   Bapcod (requires build with BapCod=ON)\n"
	    << "  -FR=[0,1]  Create real variables instead of integer variables.   \n -GAP=[0,1,2,3,4,5] Control gap for master problem\n"
	    << "  -ACO=[0,1,2,3]  Add upper bound constraint on objective function\n"
	    << "  -ACL=[0,1] Add lower bound constraint on objective function \n"
	    << "  -MS=k    Keep the k best solutions in the pool    \n -AI=[0,1] Attempt to add more improved cuts \n-H=[0,1,2]   Use Heuristic, 0: no heuristic, 1: compute heuristic solution and stop, 2: compute heuristic solution and continue\n"
	    << "  -YT=[0,1,2] Attempt to reconstruct solutions from hub infeasibility    \n -TL=   TimeLimit (sec)  \n"
	    << "  -SFC=[0,1,2]  Strenghten Feasability Cuts   \n -FF=[0,1] Generate only feasibility cuts until a solution is found\n";
}

int main(int argc, char *argv[]) {
	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "-h" || a == "--help") {
			printUsage(argv[0]);
			return 0;
		}
	}
	if (argc < 2) {
		printUsage(argv[0]);
		return EXIT_FAILURE;
	}

	// Initialize variables
	string inputFile = argv[optind];

	int ImprovedFeasCut=0;
	int MoreCuts=0;
	int SigmaCuts=0;
	int NoMaxWork=0;
	int WarmStart=0;
	int CapH=0;
	int PartialCut=0;
	int Bapcod=0;
	int FReal=0;
	int NoObj=0;
	int ToleranceOK=0;
	int Gap=0;
	int MoreZero=0;
	int TimeCode=0;
	int AddConstraintObj=0;
	int AddObjLower=0;
	int TestLogic=0;
	int SigmaUb=0;
	int LessCut=0;
	int ColdStart=0;
	int MoreSol=1;
	int AddImprove=0;
	int UseHeuristic=0;
	int YannickT=0;
	int TimeLimit=1800;
	int NewForm=0;
	int GAP0=0;
	int SameLB=100;
	int ImortanceObj=1;
	int StrenghtenFeasCut=0;
	int FeasFirst=0;
	int ForceF=0;
	bool GPS=0;
	int Skip=0;
	int CPLEXSOLVE=0;
	string Output="";
	for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        // Check if the argument is a verbosity option
        if (arg.find("-IC=") == 0) {
            // Extract verbosity value
            ImprovedFeasCut = std::stoi(arg.substr(4));
        }
		if(arg.find("-MC=") == 0) {
			MoreCuts = std::stoi(arg.substr(4));
		}
		if(arg.find("-SC=") == 0) {
			SigmaCuts = std::stoi(arg.substr(4));
		}
		if(arg.find("-NW=") == 0) {
			NoMaxWork = std::stoi(arg.substr(4));
		}
		if(arg.find("-WS=") == 0) {
			WarmStart = std::stoi(arg.substr(4));
		}
		if(arg.find("-CH=") == 0) {
			CapH = std::stoi(arg.substr(4));
		}
		if(arg.find("-PC=") == 0) {
			PartialCut = std::stoi(arg.substr(4));
		}
		if(arg.find("-BC=") == 0) {
			Bapcod = std::stoi(arg.substr(4));
			#ifndef USE_BAP
				cout<<"Error: option -BC requires Bapcod";
				return 0;
			#endif
		}
		if(arg.find("-FR=") == 0) {
			FReal = std::stoi(arg.substr(4));
		}
		if(arg.find("-NO=") == 0) {
			NoObj = std::stoi(arg.substr(4));
		}if(arg.find("-GAP=") == 0) {
			Gap = std::stoi(arg.substr(5));
		}
		if(arg.find("-MZ=") == 0) {
			MoreZero = std::stoi(arg.substr(4));
		}
		if(arg.find("-TC=") == 0) {
			TimeCode = std::stoi(arg.substr(4));
		}
		if(arg.find("-ACO=") == 0) {
			AddConstraintObj = std::stoi(arg.substr(5));
		}
		if(arg.find("-ACL=") == 0) {
			AddObjLower = std::stoi(arg.substr(5));
		}
		if(arg.find("-LC=") == 0) {
			LessCut = std::stoi(arg.substr(4));
		}
		if(arg.find("-CS=") == 0) {
			ColdStart = std::stoi(arg.substr(4));
		}
		if(arg.find("-MS=") == 0) {
			MoreSol = std::stoi(arg.substr(4));
		}
		if(arg.find("-AI=") == 0) {
			AddImprove = std::stoi(arg.substr(4));
		}
		if(arg.find("-H=") == 0) {
			UseHeuristic = std::stoi(arg.substr(3));
		}
		if(arg.find("-YT=") == 0) {
			YannickT = std::stoi(arg.substr(4));
		}
		if(arg.find("-TL=") == 0) {
			TimeLimit = std::stoi(arg.substr(4));
		}
		if(arg.find("-NF=") == 0) {
			NewForm = std::stoi(arg.substr(4));
		}
		if(arg.find("-EX=") == 0) {
			GAP0 = std::stoi(arg.substr(4));
		}if(arg.find("-OUT=") == 0) {
			Output = arg.substr(5);
		}if(arg.find("-SLB=") == 0) {
			SameLB = std::stoi(arg.substr(5));
		}if(arg.find("-IO=") == 0) {
			ImortanceObj = std::stoi(arg.substr(4));
		}
		if(arg.find("-SFC=") == 0) {
			StrenghtenFeasCut = std::stoi(arg.substr(5));
		}if(arg.find("-FF=") == 0) {
			FeasFirst = std::stoi(arg.substr(4));
		}if(arg.find("-HF=") == 0) {
			ForceF = std::stoi(arg.substr(4));
		}if(arg.find("-GPS=") == 0) {
			if(std::stoi(arg.substr(5))==1)
				GPS = true;
		}if(arg.find("-SK=") == 0) {
			Skip = std::stoi(arg.substr(4));
		}
		if(arg.find("-CPL")== 0){
			CPLEXSOLVE = std::stoi(arg.substr(5));
		}

    }
	// Initialize data containers
	// Read the file
	ifstream file(inputFile);
	if (!file.is_open()) {
		std::cerr << "Error opening file: " << inputFile << std::endl;
		return EXIT_FAILURE;
	}
	if(AddImprove==1 && ImprovedFeasCut==0){
		cout<<"Can't use option -AI without option -IC"<<endl;
	}else{
		MyInstance Inst;
		
		Inst.fromFile(inputFile,ImprovedFeasCut,MoreCuts,SigmaCuts,NoMaxWork,WarmStart,CapH,PartialCut,Bapcod,FReal,NoObj,ToleranceOK,Gap,MoreZero,TimeCode,AddConstraintObj,TestLogic,SigmaUb,LessCut,ColdStart,MoreSol,AddObjLower,AddImprove,YannickT,TimeLimit,NewForm,GAP0,Output,SameLB,ImortanceObj,StrenghtenFeasCut,FeasFirst,ForceF,GPS,Skip);
		
		// Model the problem
		int ub=1000000;
		if(UseHeuristic>=1){
			ub=FindUpper(Inst);
			string ubfile = inputFile+".UB";
			ofstream myfile;
			myfile.open (ubfile);
			myfile << ub;
			myfile.close();

		}
		if(CPLEXSOLVE==1){
			CPLEXOPTIMIZE(Inst,ub);
		}
		if(UseHeuristic!=1 && CPLEXSOLVE==0)
			mainBend(Inst,ub);	
	}
    return 0;
}
