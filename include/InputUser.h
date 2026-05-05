#ifndef VRPSTW_INPUTUSER_H
#define VRPSTW_INPUTUSER_H

#include "Data.h"
#include "Parameters.h"

namespace vrpstw
{
	class InputUser
	{
	friend class Model;
	public:
		InputUser() : data(Data::getInstance()), params(Parameters::getInstance()) {}
		Data & data;
		const Parameters & params;
		void ClearData(){
			data.nbCustomers=0;
			data.nbDepots=0;
			data.customers.clear();
			data.depots.clear();
			data.customers.push_back(vrpstw::Customer());
			data.depots.push_back(vrpstw::Depot());
		}
	};
}

#endif
