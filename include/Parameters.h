#ifndef VRPSTW_PARAMETERS_H
#define VRPSTW_PARAMETERS_H

#include "Singleton.h"
#include "bcParameterParserC.hpp"

namespace vrpstw
{
	class Parameters : public Singleton<Parameters>, ParameterParser
	{
		friend class Singleton<Parameters>;
	public:
		Parameters();
		virtual ~Parameters() {}

		bool loadParameters(const std::string & parameterFileName, int argc, char* argv[]);
		bool loadParameter(const std::string & parameterFileName);
        ApplicationParameter<double> cutOffValue;
        ApplicationParameter<bool> silent;
	};
}

#endif
