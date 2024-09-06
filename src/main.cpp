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
	for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        // Check if the argument is a verbosity option
        if (arg.find("-IFC=") == 0) {
            // Extract verbosity value
            ImprovedFeasCut = std::stoi(arg.substr(5));
        }else if(arg.find("-MC=") == 0) {
			MoreCuts = std::stoi(arg.substr(4));
		}else if(arg.find("-SC=") == 0) {
			SigmaCuts = std::stoi(arg.substr(4));
		}else if(arg.find("-NW=") == 0) {
			NoMaxWork = std::stoi(arg.substr(4));
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
	
	Inst.fromFile(inputFile,ImprovedFeasCut,MoreCuts,SigmaCuts,NoMaxWork);
	
	// Model the problem
	
	mainBend(Inst);	
    return 0;
}
