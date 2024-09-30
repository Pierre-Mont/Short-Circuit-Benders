#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <limits>
#include <unistd.h>
#include <cassert>
#include "Bender.hpp"


int main(int argc, char *argv[]) {
    
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
		}
		if(arg.find("-FR=") == 0) {
			FReal = std::stoi(arg.substr(4));
		}
		if(arg.find("-NO=") == 0) {
			NoObj = std::stoi(arg.substr(4));
		}
		if(arg.find("-TO=") == 0) {
			ToleranceOK = std::stoi(arg.substr(4));
		}
		if(arg.find("-GAP=") == 0) {
			Gap = std::stoi(arg.substr(5));
		}
		if(arg.find("-MZ=") == 0) {
			MoreZero = std::stoi(arg.substr(4));
		}
    }
	// Initialize data containers
	// Read the file
	ifstream file(inputFile);
	if (!file.is_open()) {
		std::cerr << "Error opening file: " << inputFile << std::endl;
		return EXIT_FAILURE;
	}
	MyInstance Inst;
	
	Inst.fromFile(inputFile,ImprovedFeasCut,MoreCuts,SigmaCuts,NoMaxWork,WarmStart,CapH,PartialCut,Bapcod,FReal,NoObj,ToleranceOK,Gap,MoreZero);
	
	// Model the problem
	
	mainBend(Inst);	
    return 0;
}
