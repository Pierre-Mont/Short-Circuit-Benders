#ifndef VRPSTW_LOADER_H
#define VRPSTW_LOADER_H

#include <string>
#include <vector>
namespace vrpstw
{
	class Data;
	class Parameters;

	class Loader
	{
	public:
		Loader();

		bool loadData(const std::string & file_name);
		bool loadParameters(const std::string & file_name, int argc, char* argv[]);
		bool loadParameter(const std::string & file_name);
		bool loadMyVRp(int maxNbVehicles, int capacity, int MaxWork, int nbCust, const std::vector<double>& xCoord, const std::vector<double>& yCoord, const std::vector<int>& demand, const std::vector<int>& DistDepot, bool GPSCoord);

		Data getData();
		void ClearData();
	private:
		Data & data;
		Parameters & parameters;

		bool loadVRPTWFile(std::ifstream & ifs);
	};
}

#endif
