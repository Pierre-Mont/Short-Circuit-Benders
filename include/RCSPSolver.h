#ifndef VRPSTW_RCSPSOLVER_H
#define VRPSTW_RCSPSOLVER_H

#include "InputUser.h"

#include <vector>

#include <bcModelRCSPSolver.hpp>

namespace vrpstw
{
	class RCSPSolver : public InputUser
	{
	public:
		RCSPSolver(BcFormulation spForm, int depotId);
		virtual ~RCSPSolver() {}

#ifdef BCP_RCSP_IS_FOUND
        BcRCSPFunctor * getOracle() { return oracle; }
#endif

	
		BcFormulation spForm;
		int depotId;

		std::vector<BcVertex> toVertices;
		std::vector<BcVertex> fromVertices;
#ifdef BCP_RCSP_IS_FOUND
        BcRCSPFunctor * oracle;
#endif

        void buildVertices(BcNetwork & network, BcNetworkResource & time_res, BcNetworkResource & cap_res, int depot_id);
		void buildArcs(BcNetwork & network, BcNetworkResource & time_res, BcNetworkResource & cap_res, int depot_id);
		void buildElemSetDistanceMatrix(BcNetwork & network);
	};
}

#endif
