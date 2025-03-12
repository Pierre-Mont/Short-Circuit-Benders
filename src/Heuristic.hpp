#include "MyInstance.hpp"
#include "Bender.hpp"
#include <vector>
#include <algorithm>


 pair<int,int> coutInsertion(MyInstance Inst, vector<int> CurrChemin, int newpoint){
    if(CurrChemin.size()==1){
        return {2*Inst.dist[CurrChemin[0]][newpoint],0};
    }
    if(CurrChemin.back()==newpoint){
            return {0,-2};
    }
    int mincost=1000;
    int insertplace=0;
    for (size_t i = 0; i < CurrChemin.size()-1; i++){
        if(CurrChemin[i]==newpoint){
            return {0,-2};
        }
        if(Inst.dist[CurrChemin[i]][newpoint]+Inst.dist[CurrChemin[i+1]][newpoint]-Inst.dist[CurrChemin[i+1]][CurrChemin[i]] < mincost){
            insertplace=i;
            mincost=Inst.dist[CurrChemin[i]][newpoint]+Inst.dist[CurrChemin[i+1]][newpoint]-Inst.dist[CurrChemin[i+1]][CurrChemin[i]];
        }
    }
    
    if(Inst.dist[CurrChemin.back()][newpoint]+Inst.dist[CurrChemin[0]][newpoint]-Inst.dist[CurrChemin.back()][CurrChemin[0]] < mincost){
            insertplace=CurrChemin.size()-1;
            mincost=Inst.dist[CurrChemin.back()][newpoint]+Inst.dist[CurrChemin[0]][newpoint]-Inst.dist[CurrChemin.back()][CurrChemin[0]];
    }
    assert(mincost>=0);
    return {mincost,insertplace};
}


vector<vector<array<int,6>>> coutInsertionsPick(MyInstance Inst, int currC, int currK, int early, int late,  vector<vector<vector<int>>> CurrChemin, vector<vector<int>> CurrWork,vector<vector<int>> CurrCapa, vector<vector<int>> CurrStocks,vector<vector<vector<vector<vector<int>>>>> CurrPickHub,vector<vector<vector<int>>> CurrWorkHub,vector<vector<vector<vector<int>>>> CurrCapaHub){
    vector<vector<array<int,6>>> resultat;
    pair<int,int> insert;
    for (int i = 0; i < Inst.Nh; i++){
        resultat.push_back({});
        for (int t = 0; t < Inst.Nt; t++)
        {
            //cost, origin, arriver, position,vehicle,tour
            resultat.back().push_back({1000,-1,-1,0,0,0});
        }
        
    }
    assert(late<=Inst.Nt);
    assert(currC<Inst.Nc && currK < Inst.Nk);
    for (int i = 0; i < Inst.Nh; i++){
        for (int t = early; t < late; t++){
            for (int p = 0; p < Inst.Np; p++){ 
                if(CurrStocks[p][currK] >= Inst.demands[currC][currK] && Inst.dist[p][Inst.Np+i] <= Inst.WorkHub/2){
                    for (int j = 0; j < Inst.Nvh; j++){
                        for (size_t l = 0; l < CurrPickHub[i][j][t].size(); l++){
                            if(CurrCapaHub[i][j][t][l]+Inst.demands[currC][currK]*Inst.Psize[currK]<= Inst.CapaHub){
                                insert=coutInsertion(Inst,CurrPickHub[i][j][t][l],p);
                                if(insert.first<resultat[i][t][0] && insert.first+CurrWorkHub[i][j][t]<=Inst.WorkHub){
                                    resultat[i][t][0]=insert.first;
                                    resultat[i][t][1]=i+Inst.Np;
                                    resultat[i][t][2]=p;
                                    resultat[i][t][3]=insert.second;
                                    resultat[i][t][4]=j;
                                    resultat[i][t][5]=l;

                                }
                            }
                        }
                        insert=coutInsertion(Inst,{Inst.Np+i},p);
                        if(insert.first<resultat[i][t][1] && insert.first+CurrWorkHub[i][j][t]<=Inst.WorkHub){
                            resultat[i][t][0]=insert.first;
                            resultat[i][t][1]=i+Inst.Np;
                            resultat[i][t][2]=p;
                            resultat[i][t][3]=insert.second;
                            resultat[i][t][4]=j;
                            resultat[i][t][5]=-1;
                        }
                    }
                    if(Inst.Prod_av[p][t] && Inst.dist[p][Inst.Np+i] <= 42 && CurrCapa[p][t]+Inst.demands[currC][currK]*Inst.Psize[currK]<= Inst.CapaProd){
                        insert=coutInsertion(Inst,CurrChemin[p][t],i+Inst.Np+Inst.Nh+Inst.Nc);
                        if(insert.first<resultat[i][t][0] && insert.first+CurrWork[p][t]<Inst.WorkProd){
                            resultat[i][t][0]=insert.first;
                            resultat[i][t][1]=p;
                            resultat[i][t][2]=i+Inst.Np+Inst.Nh+Inst.Nc;
                            resultat[i][t][3]=insert.second;
                        }
                    }
                }
            }
        }
    }
    
    /*for (size_t i = 0; i < resultat.size(); i++)
    {
        for (size_t j = 0; j < resultat[i].size(); j++){
            cout<<currC<<" "<<currK<<" "<<i<<" "<<j<<endl;;
             for (size_t l = 0; l < resultat[i][j].size(); l++){
                cout<<resultat[i][j][l]<<" ";
             }
             cout<<endl;
        }
    }*/
    
    return resultat;
    
}
int FindUpper(MyInstance Inst){
    vector<pair<int,int>> Mycommands;
    vector<pair<int,int>> PoolHub;
    vector<vector<int>> PossiblProd;
    vector<vector<int>> CurrStocks=Inst.stocks;
    vector<vector<vector<int>>> CurrChemin;
    vector<vector<int>> CurrWork,CurrCapa;
    vector<vector<vector<vector<vector<int>>>>> CurrPickHub,CurrDeliHub;
    vector<vector<vector<vector<int>>>> CurrCapaPickHub,CurrCapaDeliHub;

    vector<vector<vector<int>>> CurrWorkHub;
    vector<vector<vector<vector<int>>>> CommandsPickHub;
    vector<vector<vector<vector<int>>>> DemandsPickHub;
    vector<vector<vector<vector<int>>>> CommandsDeliHub;
    vector<vector<vector<vector<int>>>> DemandsDeliHub;
    vector<vector<vector<int>>> DemandsProd;
    bool atleastoneprod;
    int i,currC,currK,BestCandidate,totalwork=0;;
    pair<int,int> insert;
    vector<vector<array<int,6>>> InsertPick,InsertDeli;
    array<int,3> BestInsertion;
    array<int,4> BestInsertionHub;
    for (int i = 0; i < Inst.Np; i++){
        CurrChemin.push_back({});
        CurrWork.push_back({});
        CurrCapa.push_back({});
        DemandsProd.push_back({});
        for (int j = 0; j < Inst.Nt; j++)
        {
            DemandsProd.back().push_back(vector<int>(Inst.node,0));
            CurrWork.back().push_back(0);
            CurrCapa.back().push_back(0);
            CurrChemin.back().push_back({i});
        }
        
    }
    
    for (int c = 0; c < Inst.Nc; c++){
        for (int k = 0; k < Inst.Nk; k++){
            if(Inst.demands[c][k]>0){
                i=0;
                atleastoneprod=false;
                while(!atleastoneprod && i < Inst.Np){
                    if(Inst.stocks[i][k] >= Inst.demands[c][k] && Inst.dist[i][c+Inst.Np+Inst.Nh]<= 42 && accumulate(Inst.Prod_av[i].begin()+Inst.DeliWindowsEar[c][k]-1, Inst.Prod_av[i].begin()+Inst.DeliWindowsLat[c][k], 0)>=1){
                        atleastoneprod=true;
                    }
                    i++;
                }
                if(!atleastoneprod)
                    PoolHub.push_back({c,k});
                else
                    Mycommands.push_back({c,k});
            }
        }
    }
    /*cout<< PoolHub.size()<<endl;
     for (size_t i = 0; i < PoolHub.size(); i++)
    {
        cout<<" PoolHub "<<PoolHub[i].first<<" "<<PoolHub[i].second<<" "<< Inst.DeliWindowsEar[PoolHub[i].first][PoolHub[i].second]<<" "<<Inst.DeliWindowsLat[PoolHub[i].first][PoolHub[i].second]<<endl;
    }
    for (size_t i = 0; i < Mycommands.size(); i++)
    {
        cout<<" Mycommands "<<Mycommands[i].first<<" "<<Mycommands[i].second<<" "<< Inst.DeliWindowsEar[Mycommands[i].first][Mycommands[i].second]<<" "<<Inst.DeliWindowsLat[Mycommands[i].first][Mycommands[i].second]<<endl;
    }*/
    sort(Mycommands.begin(), Mycommands.end(), [&Inst](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        if(Inst.DeliWindowsLat[a.first][a.second]-Inst.DeliWindowsEar[a.first][a.second] == Inst.DeliWindowsLat[b.first][b.second]-Inst.DeliWindowsEar[b.first][b.second]){
                if(Inst.DeliWindowsEar[a.first][a.second]==Inst.DeliWindowsEar[b.first][b.second]){
                    return Inst.demands[a.first][a.second]*Inst.Psize[a.second] > Inst.demands[b.first][b.second]*Inst.Psize[b.second];
                }else
                    return Inst.DeliWindowsEar[a.first][a.second] < Inst.DeliWindowsEar[b.first][b.second];
        }else
            return Inst.DeliWindowsLat[a.first][a.second]-Inst.DeliWindowsEar[a.first][a.second] < Inst.DeliWindowsLat[b.first][b.second]-Inst.DeliWindowsEar[b.first][b.second];
    });
    /*cout<< Mycommands.size()<<endl;
    for (size_t i = 0; i < Mycommands.size(); i++)
    {
        cout<<" Mycommands "<<Mycommands[i].first<<" "<<Mycommands[i].second<<" "<< Inst.DeliWindowsEar[Mycommands[i].first][Mycommands[i].second]<<" "<<Inst.DeliWindowsLat[Mycommands[i].first][Mycommands[i].second]<<endl;
    }*/
    
    for (size_t i = 0; i < Mycommands.size(); i++){
        currC=Mycommands[i].first;
        currK=Mycommands[i].second;
        BestCandidate=-1;
        BestInsertion={-1,-1,1000};
        for (int j = 0; j < Inst.Np; j++){
            if(CurrStocks[j][currK]>= Inst.demands[currC][currK] && Inst.dist[j][currC+Inst.Np+Inst.Nh]<=42 ){
                for (int k = Inst.DeliWindowsEar[currC][currK]-1; k < Inst.DeliWindowsLat[currC][currK]; k++){
                    assert(k>=0);
                    if(Inst.Prod_av[j][k] && CurrCapa[j][k]+Inst.demands[currC][currK]*Inst.Psize[currK]<=Inst.CapaProd){
                        insert=coutInsertion(Inst,CurrChemin[j][k],currC+Inst.Np+Inst.Nh);
                        if(insert.first<BestInsertion[2] && insert.first+CurrWork[j][k]<Inst.WorkProd){
                            BestCandidate=j;
                            BestInsertion[2]=insert.first;
                            BestInsertion[0]=insert.second;
                            BestInsertion[1]=k;
                       }
                    }
                }
            }
        }
        if(BestCandidate!=-1){
            //cout<<"best candidtae for "<<currC<<" "<<currK<< " is producer "<<BestCandidate<<" at period "<<BestInsertion[1]<<endl;
            CurrWork[BestCandidate][BestInsertion[1]]+=BestInsertion[2];
            assert( CurrWork[BestCandidate][BestInsertion[1]]<=Inst.WorkProd);
            CurrCapa[BestCandidate][BestInsertion[1]]+=Inst.demands[currC][currK]*Inst.Psize[currK];
            assert(CurrCapa[BestCandidate][BestInsertion[1]]<=Inst.CapaProd);
            DemandsProd[BestCandidate][BestInsertion[1]][Inst.Np+Inst.Nh+currC]=Inst.demands[currC][currK]*Inst.Psize[currK];
            totalwork+=Inst.dist[BestCandidate][currC+Inst.Np+Inst.Nh];
            CurrStocks[BestCandidate][currK]-=Inst.demands[currC][currK];
            if(BestInsertion[0]!=-2){
                CurrChemin[BestCandidate][BestInsertion[1]].insert(CurrChemin[BestCandidate][BestInsertion[1]].begin()+BestInsertion[0]+1,currC+Inst.Np+Inst.Nh);
            }
        }else{
            //cout<<"No candidtae for "<<currC<<" "<<currK<<endl;
            PoolHub.push_back({currC,currK});
        }
    }
    std::sort(PoolHub.begin(), PoolHub.end(), [&Inst](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        if(Inst.DeliWindowsLat[a.first][a.second]-Inst.DeliWindowsEar[a.first][a.second] == Inst.DeliWindowsLat[b.first][b.second]-Inst.DeliWindowsEar[b.first][b.second]){
                if(Inst.DeliWindowsEar[a.first][a.second]==Inst.DeliWindowsEar[b.first][b.second]){
                    return Inst.demands[a.first][a.second]*Inst.Psize[a.second] > Inst.demands[b.first][b.second]*Inst.Psize[b.second];
                }else
                    return Inst.DeliWindowsEar[a.first][a.second] < Inst.DeliWindowsEar[b.first][b.second];
        }else
            return Inst.DeliWindowsLat[a.first][a.second]-Inst.DeliWindowsEar[a.first][a.second] < Inst.DeliWindowsLat[b.first][b.second]-Inst.DeliWindowsEar[b.first][b.second];
    });

    for (int i = 0; i < Inst.Nh; i++){
        CurrPickHub.push_back({});
        CurrDeliHub.push_back({});
        CurrWorkHub.push_back({});
        CurrCapaPickHub.push_back({});
        CurrCapaDeliHub.push_back({});
        DemandsPickHub.push_back({});
        DemandsDeliHub.push_back({});
        CommandsPickHub.push_back({});
        CommandsDeliHub.push_back({});
        for (int j = 0; j < Inst.Nvh; j++){
            CurrPickHub.back().push_back({});
            CurrDeliHub.back().push_back({});
            CurrWorkHub.back().push_back({});
            CurrCapaPickHub.back().push_back({});
            CurrCapaDeliHub.back().push_back({});
            DemandsPickHub.back().push_back({});
            DemandsDeliHub.back().push_back({});
            CommandsPickHub.back().push_back({});
            CommandsDeliHub.back().push_back({});
            for (int t = 0; t < Inst.Nt; t++){
                DemandsPickHub.back().back().push_back(vector<int>(Inst.node,0));
                DemandsDeliHub.back().back().push_back(vector<int>(Inst.node,0));
                CommandsPickHub.back().back().push_back({});
                CommandsDeliHub.back().back().push_back({});
                CurrWorkHub.back().back().push_back(0);
                CurrCapaPickHub.back().back().push_back({0});
                CurrCapaDeliHub.back().back().push_back({0});
                CurrPickHub.back().back().push_back({{i+Inst.Np}});
                CurrDeliHub.back().back().push_back({{i+Inst.Np}});
            }
        } 
    }
    for (size_t i = 0; i < PoolHub.size(); i++){
        currC=PoolHub[i].first;
        currK=PoolHub[i].second;
        //cout<<"Consider command "<<currC<<" "<<currK<<endl;
        //assert(Inst.DeliWindowsLat[currC][currK]!=1);
        InsertPick=coutInsertionsPick(Inst,currC,currK,max(Inst.DeliWindowsEar[currC][currK]-Inst.Experiation_date[currK],1)-1,Inst.DeliWindowsLat[currC][currK]-1,CurrChemin,CurrWork,CurrCapa,CurrStocks,CurrPickHub,CurrWorkHub,CurrCapaPickHub);
        InsertDeli.clear();
        for (int i = 0; i < Inst.Nh; i++){
            InsertDeli.push_back({});
            for (int t = 0; t < Inst.Nt; t++){
                //cost, origin, arriver, position,vehicle,tour
                InsertDeli.back().push_back({1000,-1,-1,-1,0,0});
            }
        }
        //cout<<Inst.DeliWindowsEar[currC][currK]<<" "<< Inst.DeliWindowsLat[currC][currK]<<endl;
        for (int h = 0; h< Inst.Nh ; h++){
            for (int t = Inst.DeliWindowsEar[currC][currK]-1;  t < Inst.DeliWindowsLat[currC][currK]; t++){
                for (int v = 0; v < Inst.Nvh; v++){
                    for (size_t l = 0; l < CurrDeliHub[h][v][t].size(); l++){
                        if(CurrCapaDeliHub[h][v][t][l]+Inst.demands[currC][currK]*Inst.Psize[currK]<= Inst.CapaHub){
                            insert=coutInsertion(Inst,CurrDeliHub[h][v][t][l],currC+Inst.Np+Inst.Nh);
                            if(insert.first<InsertDeli[h][t][0] && insert.first+CurrWorkHub[h][v][t]<Inst.WorkHub){
                                InsertDeli[h][t][0]=insert.first;
                                InsertDeli[h][t][1]=h+Inst.Np;
                                InsertDeli[h][t][2]=currC+Inst.Np+Inst.Nh;
                                InsertDeli[h][t][3]=insert.second;
                                InsertDeli[h][t][4]=v;
                                InsertDeli[h][t][5]=l;

                            }
                        }
                    }
                    insert=coutInsertion(Inst,{Inst.Np+h},currC+Inst.Np+Inst.Nh);
                    if(insert.first<InsertDeli[h][t][0] && insert.first+CurrWorkHub[h][v][t]<=Inst.WorkHub){
                        InsertDeli[h][t][0]=insert.first;
                        InsertDeli[h][t][1]=h+Inst.Np;
                        InsertDeli[h][t][2]=currC+Inst.Np+Inst.Nh;
                        InsertDeli[h][t][3]=insert.second;
                        InsertDeli[h][t][4]=v;
                        InsertDeli[h][t][5]=-1;
                    }
                }
            
            }
        }
        /*cout<<"DELI "<<endl;
        for (size_t i = 0; i < InsertDeli.size(); i++)
        {
             for (size_t j = 0; j < InsertDeli[i].size(); j++)
            {
                cout<<InsertDeli[i][j][0]<<" "<<InsertDeli[i][j][1]<<" "<<InsertDeli[i][j][2]<<" "<<InsertDeli[i][j][3]<<" "<<InsertDeli[i][j][4]<<" "<<InsertDeli[i][j][5]<<endl;
            }
        }
        cout<<"PICJ "<<endl;
        for (size_t i = 0; i < InsertPick.size(); i++)
        {
             for (size_t j = 0; j < InsertPick[i].size(); j++)
            {
                cout<<InsertPick[i][j][0]<<" "<<InsertPick[i][j][1]<<" "<<InsertPick[i][j][2]<<" "<<InsertPick[i][j][3]<<" "<<InsertPick[i][j][4]<<" "<<InsertPick[i][j][5]<<endl;
            }
        }*/
        //Cost , hub, t, tt
        BestInsertionHub={1000,-1,-1,-1};
        for (int t = max(Inst.DeliWindowsEar[currC][currK]-Inst.Experiation_date[currK],1)-1;  t < Inst.DeliWindowsLat[currC][currK]; t++){
            for (int tt =  Inst.DeliWindowsEar[currC][currK]-1; tt < Inst.DeliWindowsLat[currC][currK]; tt++){
                if(tt > t && tt <= t+Inst.Experiation_date[currK]){
                    for (int h = 0; h < Inst.Nh; h++){
                        if(InsertPick[h][t][0]+InsertDeli[h][tt][0]< BestInsertionHub[0]){
                            BestInsertionHub[0]=InsertPick[h][t][0]+InsertDeli[h][tt][0];
                            BestInsertionHub[1]=h;
                            BestInsertionHub[2]=t;
                            BestInsertionHub[3]=tt;
                        }
                    }
                }
            }
        }
        if(BestInsertionHub[1]!=-1){
            int hub=BestInsertionHub[1];
            int period=BestInsertionHub[2];
            //cout<<"Command "<<currC<<" "<<currK<<" is fullfil by producer "<< min(InsertPick[hub][period][1],InsertPick[hub][period][2])<<" and moved from "<<InsertPick[hub][period][1]<<" to "<<InsertPick[hub][period][2]<<" during  period "<<period<<" using vehicle "<<InsertPick[hub][period][1]<<endl;
            //cout<<"Command "<<currC<<" "<<currK<<" is delivered druing period "<<BestInsertionHub[3]<<" by "<<InsertDeli[hub][BestInsertionHub[3]][1]<<" "<<endl;
            CurrStocks[min(InsertPick[hub][period][1],InsertPick[hub][period][2])][currK]-=Inst.demands[currC][currK];
            totalwork+=Inst.dist[currC+Inst.Np+Inst.Nh][min(InsertPick[hub][period][1],InsertPick[hub][period][2])];
            if(InsertPick[hub][period][1]<Inst.Np){
                DemandsProd[InsertPick[hub][period][1]][period][InsertPick[hub][period][2]]=Inst.demands[currC][currK]*Inst.Psize[currK];
                CurrCapa[InsertPick[hub][period][1]][period]+=Inst.demands[currC][currK]*Inst.Psize[currK];
                assert(CurrCapa[InsertPick[hub][period][1]][period]<=Inst.CapaProd);
                CurrWork[InsertPick[hub][period][1]][period]+=InsertPick[hub][period][0];
                assert(CurrWork[InsertPick[hub][period][1]][period]<= Inst.WorkProd);
                assert(InsertPick[hub][period][2]==hub+Inst.Np+Inst.Nc+Inst.Nh);
                if(InsertPick[hub][period][3]!=-2)
                    CurrChemin[InsertPick[hub][period][1]][period].insert(CurrChemin[InsertPick[hub][period][1]][period].begin()+InsertPick[hub][period][3]+1,InsertPick[hub][period][2]);
            }else{
                CurrWorkHub[InsertPick[hub][period][1]-Inst.Np][InsertPick[hub][period][4]][period]+=InsertPick[hub][period][0];
                assert(CurrWorkHub[InsertPick[hub][period][1]-Inst.Np][InsertPick[hub][period][4]][period]<= Inst.WorkHub);
                //CommandsPickHub[InsertPick[hub][period][1]-Inst.Np][InsertPick[hub][period][4]][period][BestInsertionHub[0]]=InsertPick[hub][period][2];
                DemandsPickHub[InsertPick[hub][period][1]-Inst.Np][InsertPick[hub][period][4]][period][InsertPick[hub][period][1]]+=Inst.demands[currC][currK]*Inst.Psize[currK];
                if(InsertPick[hub][period][5]!=-1){
                    CurrCapaPickHub[InsertPick[hub][period][1]-Inst.Np][InsertPick[hub][period][4]][period][InsertPick[hub][period][5]]+=Inst.demands[currC][currK]*Inst.Psize[currK];
                    assert(CurrCapaPickHub[InsertPick[hub][period][1]-Inst.Np][InsertPick[hub][period][4]][period][InsertPick[hub][period][5]]<=Inst.CapaHub);
                    if(InsertPick[hub][period][3]!=-2)
                        CurrPickHub[InsertPick[hub][period][1]-Inst.Np][InsertPick[hub][period][4]][period][InsertPick[hub][period][5]].insert(CurrPickHub[InsertPick[hub][period][1]-Inst.Np][InsertPick[hub][period][4]][period][InsertPick[hub][period][5]].begin()+InsertPick[hub][period][3]+1,InsertPick[hub][period][2]);
                }else{
                    CurrCapaPickHub[InsertPick[hub][period][1]-Inst.Np][InsertPick[hub][period][4]][period].push_back(Inst.demands[currC][currK]*Inst.Psize[currK]);
                    assert(Inst.demands[currC][currK]*Inst.Psize[currK]<=Inst.CapaHub);
                    if(InsertPick[hub][period][3]!=-2)
                        CurrPickHub[InsertPick[hub][period][1]-Inst.Np][InsertPick[hub][period][4]][period].push_back({hub+Inst.Np,InsertPick[hub][period][2]});
                }
            }
            period=BestInsertionHub[3];
            //CommandsDeliHub[InsertDeli[hub][period][1]-Inst.Np][InsertDeli[hub][period][4]][period].push_back(InsertDeli[hub][period][2]);
            DemandsDeliHub[InsertDeli[hub][period][1]-Inst.Np][InsertDeli[hub][period][4]][period][InsertDeli[hub][period][2]]=Inst.demands[currC][currK]*Inst.Psize[currK];
            CurrWorkHub[InsertDeli[hub][period][1]-Inst.Np][InsertDeli[hub][period][4]][period]+=InsertDeli[hub][period][0];
            assert(CurrWorkHub[InsertDeli[hub][period][1]-Inst.Np][InsertDeli[hub][period][4]][period]<=Inst.WorkHub);
            assert(InsertDeli[hub][period][1]==hub+Inst.Np);
            if(InsertDeli[hub][period][5]!=-1){
                CurrCapaDeliHub[InsertDeli[hub][period][1]-Inst.Np][InsertDeli[hub][period][4]][period][InsertDeli[hub][period][5]]+=Inst.demands[currC][currK]*Inst.Psize[currK];
                assert(CurrCapaDeliHub[InsertDeli[hub][period][1]-Inst.Np][InsertDeli[hub][period][4]][period][InsertDeli[hub][period][5]]<=Inst.CapaHub);
                if(InsertDeli[hub][period][3]!=-2)
                    CurrDeliHub[InsertDeli[hub][period][1]-Inst.Np][InsertDeli[hub][period][4]][period][InsertDeli[hub][period][5]].insert(CurrDeliHub[InsertDeli[hub][period][1]-Inst.Np][InsertDeli[hub][period][4]][period][InsertDeli[hub][period][5]].begin()+InsertDeli[hub][period][3]+1,InsertDeli[hub][period][2]);
            }else{
                CurrCapaDeliHub[InsertDeli[hub][period][1]-Inst.Np][InsertDeli[hub][period][4]][period].push_back(Inst.demands[currC][currK]*Inst.Psize[currK]);
                assert(Inst.demands[currC][currK]*Inst.Psize[currK]<= Inst.CapaHub);
                if(InsertDeli[hub][period][3]!=-2)
                    CurrDeliHub[InsertDeli[hub][period][1]-Inst.Np][InsertDeli[hub][period][4]][period].push_back({hub+Inst.Np,InsertDeli[hub][period][2]});
            }
            
        }else{
            //cout<<"COmmand "<<currC<<" "<<currK<<" is buggy"<<endl;
            totalwork+=10000;
        }
    }
    
    for (int i = 0; i < Inst.Np; i++){
       for (int t = 0; t < Inst.Nt; t++){
            if(CurrChemin[i][t].size()>1){
                totalwork+=CurrWork[i][t]; 
                //cout<<"Producer "<<i<<" visits clients "<<CurrChemin[i][t]<<" during period "<< t<<endl;
                for (size_t j = 0; j < CurrChemin[i][t].size(); j++){
                    if(i!=CurrChemin[i][t][j])
                        totalwork+=Inst.dist[i][CurrChemin[i][t][j]];
                    assert(Inst.dist[i][CurrChemin[i][t][j]] <= 42);
                }
            }else
                assert(CurrWork[i][t]==0);
       }
    }
    for (int i = 0; i < Inst.Nh; i++){
        for (int t = 0; t < Inst.Nt; t++){
            for (int v = 0; v < Inst.Nvh; v++){
                totalwork+=CurrWorkHub[i][v][t];
                /*for (size_t l = 0; l < CurrPickHub[i][v][t].size(); l++){
                    if(CurrPickHub[i][v][t][l].size()>1){
                        cout<<"HUb "<<i<<" visits producers "<<CurrPickHub[i][v][t][l]<<" during period "<< t<<" with vehicle "<<v<<endl;
                    }
                }
                for (size_t l = 0; l < CurrDeliHub[i][v][t].size(); l++){
                    if(CurrDeliHub[i][v][t][l].size()>1)
                        cout<<"HUb "<<i<<" visits clients "<<CurrDeliHub[i][v][t][l]<<" during period "<< t<<" with vehicle "<<v<<endl;
                }*/
            }
        }
    }
    /*vector<int> NodeSub,dem;
    IloEnv env;	
    IloModel WorkerModel(env);
    IloArray<IloArray<IloNumVar>> u(env, Inst.node);
    IloArray<IloArray<IloArray<IloBoolVar>>> x(env, Inst.node);
    createWorkerModel(env,WorkerModel,Inst,u,x);
    
    IloCplex WorkerCplex(WorkerModel);
    
    WorkerCplex.setParam(IloCplex::Param::Threads, 1);
    WorkerCplex.setParam(IloCplex::Param::MIP::Display, 0);
    WorkerCplex.setOut(env.getNullStream());  // Suppress log output
    WorkerCplex.setParam(IloCplex::IloCplex::Param::MIP::Tolerances::Integrality, 1e-9);  // Integer feasibility tolerance
    WorkerCplex.setParam(IloCplex::Param::Simplex::Tolerances::Optimality, 1e-9);  // Optimality tolerance
    WorkerCplex.setParam(IloCplex::Param::Feasopt::Tolerance, 1e-9);
    WorkerCplex.setWarning(env.getNullStream());

    for (int i = 0; i < Inst.Np; i++){
        for (int t = 0; t < Inst.Nt; t++){
            if(CurrChemin[i][t].size()>1){
                NodeSub=CurrChemin[i][t];
                dem=DemandsProd[i][t];
                IloConstraintArray cons(WorkerModel.getEnv());
                for (IloModel::Iterator it(WorkerModel); it.ok(); ++it) {
                    if ( (*it).isConstraint() )
                        cons.add((*it).asConstraint());
                }
                WorkerModel.remove(cons);
                cons.endElements();
                cons.end();
                int tour=1;
                assert(accumulate(dem.begin(), dem.end(), 0)<= Inst.CapaProd);
                GenWorkerModel(env, WorkerModel,Inst,u,x,NodeSub,dem, tour,Inst.WorkProd*100,Inst.CapaProd);
                WorkerCplex.solve();
                assert(WorkerCplex.getStatus()==IloAlgorithm::Optimal);
                assert(WorkerCplex.getObjValue()-CurrWork[i][t]<=0+0.1);
                assert(WorkerCplex.getObjValue()-Inst.WorkProd<= 0.1);
            }
        } 
    }
    int res,res2;
    for (int h = 0; h < Inst.Nh; h++){
        for (int v = 0; v < Inst.Nvh; v++){
            for (int t = 0; t < Inst.Nt; t++){
                res=0;
                res2=0;
                if(CommandsDeliHub[h][v][t].size()>0){
                    NodeSub.clear();
                    sort(CommandsDeliHub[h][v][t].begin(),CommandsDeliHub[h][v][t].end());
                    NodeSub.push_back(CommandsDeliHub[h][v][t][0]);
                    for (size_t j = 1; j < CommandsDeliHub[h][v][t].size(); j++){
                        if(CommandsDeliHub[h][v][t][j]!=NodeSub.back())
                            NodeSub.push_back(CommandsDeliHub[h][v][t][j]);
                    }
                    NodeSub.insert(NodeSub.begin(),h+Inst.Np);
                    dem=DemandsDeliHub[h][v][t];

                    IloConstraintArray cons(WorkerModel.getEnv());
                    for (IloModel::Iterator it(WorkerModel); it.ok(); ++it) {
                        if ( (*it).isConstraint() )
                            cons.add((*it).asConstraint());
                    }
                    WorkerModel.remove(cons);
                    cons.endElements();
                    cons.end();


                    int tour=NodeSub.size()-1;                    
                    GenWorkerModel(env, WorkerModel,Inst,u,x,NodeSub,dem, tour,Inst.WorkHub,Inst.CapaHub);			
                    cout<<NodeSub<<endl;
                    cout<<"dem "<<dem<<endl;
                    WorkerCplex.exportModel("filework.lp");
                    WorkerCplex.solve();
                    assert(WorkerCplex.getStatus()==IloAlgorithm::Optimal);
                    res=WorkerCplex.getObjValue();
                }
                if(CommandsPickHub[h][v][t].size()>0){
                    NodeSub.clear();
                    sort(CommandsPickHub[h][v][t].begin(),CommandsPickHub[h][v][t].end());
                    NodeSub.push_back(CommandsPickHub[h][v][t][0]);
                    for (size_t j = 1; j < CommandsPickHub[h][v][t].size(); j++){
                        if(CommandsPickHub[h][v][t][j]!=NodeSub.back())
                            NodeSub.push_back(CommandsPickHub[h][v][t][j]);
                    }
                    NodeSub.insert(NodeSub.begin(),h+Inst.Np);
                    dem=DemandsPickHub[h][v][t];
                    IloConstraintArray cons(WorkerModel.getEnv());
                    for (IloModel::Iterator it(WorkerModel); it.ok(); ++it) {
                        if ( (*it).isConstraint() )
                            cons.add((*it).asConstraint());
                    }
                    WorkerModel.remove(cons);
                    cons.endElements();
                    cons.end();


                    int tour=ceil(accumulate(dem.begin(), dem.end(), 0)/(double)Inst.CapaHub);
                    
                    GenWorkerModel(env, WorkerModel,Inst,u,x,NodeSub,dem, tour,Inst.WorkHub,Inst.CapaHub);	
                    WorkerCplex.solve();			
                    assert(WorkerCplex.getStatus()==IloAlgorithm::Optimal);
                    //cout<<WorkerCplex.getObjValue()<<" "<<CurrWorkHub[h][v][t]<<" "<<res<<endl;
                    
                    res2=WorkerCplex.getObjValue();
                }
                assert(res2+res-CurrWorkHub[h][v][t]<=0.1);
                assert(res2+res-Inst.WorkHub<= 0.1);
            }
        } 
    }
    WorkerCplex.end();
    WorkerModel.end();
    env.end();*/
    cout<<"Heuristic Solution is "<<totalwork<<endl;
    return totalwork;
}