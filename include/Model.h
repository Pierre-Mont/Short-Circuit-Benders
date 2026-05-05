#ifndef VRPSTW_MODEL_H
#define VRPSTW_MODEL_H

#include <bcModelPointerC.hpp>
#include "InputUser.h"

namespace vrpstw
{
	class Model : public BcModel, InputUser
	{
	private: 
		BcVarArray xvar;
	public:
		Model(const BcInitialisation& bc_init);
		virtual ~Model() {}
		const Data& getData() const { return data; }
		void ClearData(){
			data.nbCustomers=0;
			data.nbDepots=0;
			data.customers.clear();
			data.depots.clear();
			data.customers.push_back(vrpstw::Customer());
			data.depots.push_back(vrpstw::Depot());
		}
		BcVarArray& getVariables() {
        	return xvar;
		}
	};
}

#endif
