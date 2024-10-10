#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;


float round2d(float var)
{
    // 37.66666 * 100 =3766.66
    // 3766.66 + .5 =3767.16    for rounding off value
    // then type cast to int so value is 3767
    // then divided by 100 so the value converted into 37.67
    float value = (int)(var * 100 + .5);
    return (float)value / 100;
}
class MyInstance{
	public:
	int Np, Nc, Nh, Nk, Nvh, CapaProd, WorkProd,CapaHub, WorkHub, Nt, TourHub, node, Nv, NbOptCut, NbFeasCut,NbNodeSubs,NbSolvedSubs,MaxNode,MinNode,CapH,PartialCut,Bapcod,FReal,NoObj,ToleranceOK,Gap,MoreZero,TimeCode,AddConstraintObj,TestLogic,SigmaUb;
	vector<float> MinDist;
	vector<vector<float>> dist;
	//For Debugging
	vector<int> x_all, y_all; // x_c, y_c, x_p, y_p, x_h, y_h;
	vector<int>  Experiation_date, Psize,  Pplus ,Cmoins, StartVehicle, CapaVehicle, WorkVehicle, TourVehicle;
	vector<vector<int>> stocks, Clients ,demands, DeliWindowsEar, DeliWindowsLat,Vehicles;
	vector<vector<bool>> Prod_av, Client_av;
	vector<pair<int,int>> PairHub;
	int ImprovedCut,MoreCuts,SigmaCuts,NoMaxWork,WarmStart;
	string intputFile;

	void fromFile(const std::string& inputFile_in,int ImprovedFeasCut_in, int MoreCuts_in, int SigmaCuts_in, int NoMaxWork_in, int WarmStart_in,int CapH_in, int PartialCut_in, int Bapcod_in, int FReal_in, int NoObj_in,int ToleranceOK_in,int Gap_in, int MoreZero_in, int TimeCode_in, int AddConstraintObj_in,int TestLogic_in,int SigmaUb_in) {
        ifstream file(inputFile_in);
		if (!file.is_open()) {
			std::cerr << "Error opening file: " << inputFile_in << std::endl;
		}
		intputFile=inputFile_in+".TC.txt";

		int count = 0;
		string line;
		NbOptCut=0;
		MaxNode=0;
		MinNode=1000;
		ImprovedCut=ImprovedFeasCut_in;
		MoreCuts=MoreCuts_in;
		SigmaCuts=SigmaCuts_in;
		NoMaxWork=NoMaxWork_in;
		WarmStart=WarmStart_in;
		PartialCut=PartialCut_in;
		Bapcod=Bapcod_in;
		CapH=CapH_in;
		FReal=FReal_in;
		NoObj=NoObj_in;
		ToleranceOK=ToleranceOK_in;
		Gap=Gap_in;
		TimeCode=TimeCode_in;
		NbNodeSubs=0;
		NbSolvedSubs=0;
		NbFeasCut=0;
		AddConstraintObj=AddConstraintObj_in;
		MoreZero=MoreZero_in;
		TestLogic=TestLogic_in;
		SigmaUb=SigmaUb_in;
        while (getline(file, line)) {
			istringstream iss(line);
			if (count == 0) {
				iss >>  Np >>  Nc >> Nh >>  Nk >>  Nvh >>  CapaProd >>  WorkProd >>  CapaHub >>  WorkHub >>  Nt >> TourHub ;
				node = Np + Nc + 2 * Nh;
				Nv = Nvh * Nh + Np;
			}else if(count == 1){
				int re;
				for (int i = 0; i < Np; i++)
				{
					stocks.push_back({});
					for (int j = 0; j < Nk; j++){
						iss >> re;
						stocks.back().push_back(re);
					}
				}
			}else if(count == 2){
				int re;
				for (int i = 0; i < Nc; i++)
				{
					demands.push_back({});
					for (int j = 0; j < Nk; j++){
						iss >> re;
						demands.back().push_back(re);
					}
				}
			}else if(count == 3){
				bool re;
				for (int i = 0; i < Np; i++)
				{
					Prod_av.push_back({});
					for (int j = 0; j < Nt; j++){
						iss >> re;
						Prod_av.back().push_back(re);
					}
				}
			}else if(count == 4){
				bool re;
				for (int i = 0; i < Nc; i++)
				{
					Client_av.push_back({});
					for (int j = 0; j < Nt; j++){
						iss >> re;
						Client_av.back().push_back(re);
					}
				}
			}else if(count == 5){
				int re;
				for (int i = 0; i < Nc; i++)
				{
					DeliWindowsEar.push_back({});
					DeliWindowsLat.push_back({});
					for (int j = 0; j < Nk; j++){
						iss >> re;
						DeliWindowsEar.back().push_back(re);
						iss >> re;
						DeliWindowsLat.back().push_back(re);
					}
				}
			}else if(count == 6){
				int re;
				for (int i = 0; i < Nk; i++)
				{
					iss >> re;
					Experiation_date.push_back(re);	
				}
			}else if(count == 7){
				float re, MinD;
				for (int i = 0; i < node; i++)
				{
					MinD=10000000;
					dist.push_back({});
					for (int j = 0; j < node; j++){
						iss >> re;
						if(i!=j && MinD>re){
							if(i<Np || (i>=Np+Nh && i<Np+Nh+Nc) || (j!=i+Nh+Nc && j!=i-Nh-Nc))
								MinD=re;
						}
						dist.back().push_back(re);
					}
					MinDist.push_back(MinD);
				}
			}else if(count == 8){
				int re;
				for (int i = 0; i < Nk; i++)
				{
					iss >> re;
					Psize.push_back(re);	
				}
			}else if(count == 9){
				int re;
				for (int i = 0; i < node; i++)
				{
					iss >> re;
					x_all.push_back(re);
					iss >> re;
					y_all.push_back(re);	
				}
			}
			count++;
		}
		for (int i = 0; i < Np; i++) {
			Vehicles.push_back({i});
			CapaVehicle.push_back(CapaProd);
			WorkVehicle.push_back(WorkProd);
			StartVehicle.push_back(i);
			TourVehicle.push_back(1);
			Pplus.push_back(i);
		}
		for (int i = 0; i < Nc; i++){
			Cmoins.push_back(i+Np+Nh);
		}
		
		int numvehi=Np;
		for (int i = 0; i < Nh; i++){
			Vehicles.push_back({});
			Pplus.push_back(i+Np);
			PairHub.push_back({0,0});
			PairHub.back().first=i+Np;
			for (int j = 0; j < Nvh; j++){
				StartVehicle.push_back(i+Np);
				Vehicles.back().push_back(numvehi);
				CapaVehicle.push_back(CapaHub);
				WorkVehicle.push_back(WorkHub);
				TourVehicle.push_back(TourHub);
				numvehi++;
			}
			Cmoins.push_back(i+Nc+Np+Nh);
			PairHub.back().second=i+Nc+Np+Nh;
		}
    }
};