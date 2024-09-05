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

	// Initialize data containers
	// Read the file
	ifstream file(inputFile);
	if (!file.is_open()) {
		std::cerr << "Error opening file: " << inputFile << std::endl;
		return EXIT_FAILURE;
	}
	MyInstance Inst;
	
	Inst.fromFile(inputFile);
	
	// Model the problem
	
	mainBend(Inst);	
    return 0;
}
