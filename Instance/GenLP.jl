# Copyright 2017, Iain Dunning, Joey Huchette, Miles Lubin, and contributors    #src
# This Source Code Form is subject to the terms of the Mozilla Public License   #src
# v.2.0. If a copy of the MPL was not distributed with this file, You can       #src
# obtain one at https://mozilla.org/MPL/2.0/.                                   #src

# # The facility location problem

# **This tutorial was originally contributed by Mathieu Tanneau and Alexis Montoison.**

# ## Required packages

# This tutorial requires the following packages:


using JuMP
import HiGHS
import LinearAlgebra
using Printf
import Random
using Dates
# ## Uncapacitated facility location

# ### Problem description
#
# We are given
# * A set $M=\{1, \dots, m\}$ of clients
# * A set $N=\{ 1, \dots, n\}$ of sites where a facility can be built
#
# **Decision variables**
# Decision variables are split into two categories:
# * Binary variable $y_{j}$ indicates whether facility $j$ is built or not
# * Binary variable $x_{i, j}$ indicates whether client $i$ is assigned to facility $j$
#
# **Objective**
# The objective is to minimize the total cost of serving all clients.
# This costs breaks down into two components:
# * Fixed cost of building a facility.
# In this example, this cost is $f_{j} = 1, \ \forall j$.
# * Cost of serving clients from the assigned facility.
# In this example, the cost $c_{i, j}$
# of serving client $i$ from facility $j$
# is the Euclidean distance between the two.
#
# **Constraints**
# * Each customer must be served by exactly one facility
# * A facility cannot serve any client unless it is open

# ### MILP formulation
#
# The problem can be formulated as the following MILP:
#
# ```math
# \begin{aligned}
# \min_{x, y} \ \ \ &
# \sum_{i, j} c_{i, j} x_{i, j} +
# \sum_{j} f_{j} y_{j} \\
# s.t. &
# \sum_{j} x_{i, j} = 1, && \forall i \in M \\
# & x_{i, j} \leq y_{j}, && \forall i \in M, j \in N \\
# & x_{i, j}, y_{j} \in \{0, 1\}, && \forall i \in M, j \in N
# \end{aligned}
# ```
#
# where the first set of constraints ensures
# that each client is served exactly once,
# and the second set of constraints ensures
# that no client is served from an unopened facility.

# ### Problem data

# To ensure reproducibility, we set the random number seed:

start_time = now()




if length(ARGS) < 1
    se=5
    Random.seed!(se)
    # Here's the data we need:
    ## Number of Producers
    Np = 3
    ## Number of Customers
    Nc = 8
    ## Number of Hubs
    Nh = 1
    ## Number of Products
    Nk = 2
    ## Number of Periods
    Nt = 5
    ## Number of vehicle per hubs
    Nvh = 1
    ## Number of allowed tour per period
    Nvr = 2
    ## Number of vehicles
    

    CapaProd=50
    CapaHub=60
    WorkProd=30
    WorkHub=60
    TourHub=3
    MaxSize=3
    MaxCo=10
    MaxStock=20
    MaxDemand=3
else
    # Get the filename from command-line arguments
    filename = ARGS[1]

    # Open the file in read mode
    file = open(filename, "r")

    # Read the first line and parse the integers
    line1 = readline(file)
    line1 = readline(file)
    values1 = parse.(Int, split(line1))
    se, Nt, MaxCo, Nk, MaxSize=values1
    
    # Read the second line and parse the integers
    line2 = readline(file)
    line2 = readline(file)
    values2 = parse.(Int, split(line2))
    Np, CapaProd, WorkProd, MaxStock=values2

    line3 = readline(file)
    line3 = readline(file)
    values3 = parse.(Int, split(line3))
    Nc, MaxDemand=values3 
    # Close the file
    line4 = readline(file)
    line4 = readline(file)
    values4 = parse.(Int, split(line4))
    Nh, Nvh, CapaHub, WorkHub, TourHub= values4
    Random.seed!(se)
    close(file)

    # Assign the values to variables
    #a, b, c, d, e = values1
    #x, y, z, w = values2
end
Nv = Nvh*Nh + Np
node=Np+Nc+2*Nh
## Clients' locations
x_c, y_c = rand(Nc), rand(Nc)

## Producer' potential locations
x_p, y_p = rand(Np), rand(Np)

## Hub' potential locations
x_h, y_h = rand(Nh), rand(Nh)

## Node position
x_all, y_all=rand(0:MaxCo,Np+Nh+Nc+Nh), rand(0:MaxCo,Np+Nh+Nc+Nh)
## Distance
dist = zeros(node, node)
dist2 = zeros(node, node)
for i in 1:Np+Nc+Nh
    for j in 1:Np+Nc+Nh
        dist2[i, j] = LinearAlgebra.norm([x_all[i] - x_all[j], y_all[i] - y_all[j]], 2)
    end
    for j in 1:Nh
        dist2[i, j+Np+Nc+Nh] = LinearAlgebra.norm([x_all[i] - x_all[j+Np], y_all[i] - y_all[j+Np]], 2)
    end
end
for i in 1:Nh
    for j in 1:Np+Nc+Nh
        dist2[i+Np+Nc+Nh, j] = LinearAlgebra.norm([x_all[i+Np] - x_all[j], y_all[i+Np] - y_all[j]], 2)
    end
    for j in 1:Nh
        dist2[i+Np+Nc+Nh, j+Np+Nc+Nh] = LinearAlgebra.norm([x_all[i+Np] - x_all[j+Np], y_all[i+Np] - y_all[j+Np]], 2)
    end
end
rounded_matrix = round.(dist2, digits=2)

dist2=rounded_matrix

## List of clients, producers, hubs, vehicles
ClientsF=zeros(Nh+Np)
Clients = round.(Int, ClientsF)
Vehicles=[]
StartVehicle=[]
CapaVehicle=[]
WorkVehicle=[]
TourVehicle=[]
PplusF=zeros(Nh+Np)
Pplus = round.(Int, PplusF)

CmoinsF=zeros(Nh+Nc)
Cmoins = round.(Int, CmoinsF)

PairHubF=zeros(Nh,2)
PairHub = round.(Int, PairHubF)
for i in 1:Np
    x_p[i]=x_all[i]
    y_p[i]=y_all[i]
    push!(Vehicles,[i])
    push!(CapaVehicle,CapaProd)
    push!(WorkVehicle,WorkProd)
    push!(StartVehicle,i)
    push!(TourVehicle,1)
    Pplus[i]=i
end
global numvehi=Np+1
for i in 1:Nh
    Pplus[i+Np]=i+Np
    PairHub[i,1]=i+Np
    x_h[i]=x_all[i+Np]
    y_h[i]=y_all[i+Np]
    x_all[i+Np+Nc+Nh]=x_all[i+Np]
    y_all[i+Np+Nc+Nh]=y_all[i+Np]
    addVehi=[]
    for j in 1:Nvh
        push!(StartVehicle,i+Np)
        push!(addVehi,numvehi)
        push!(TourVehicle,TourHub)
        push!(WorkVehicle,WorkHub)
        push!(CapaVehicle,CapaHub)
        global numvehi+=1
    end
    push!(Vehicles,addVehi)
end
for i in 1:Nc
    x_c[i]=x_all[i+Np+Nh]
    y_c[i]=y_all[i+Np+Nh]
    Cmoins[i]=i+Np+Nh
end
for i in 1:Nh
    Cmoins[i+Nc]=i+Nc+Np+Nh
    PairHub[i,2]=i+Nc+Np+Nh
end

## Create the demands and stock, verify that sum of demands is inferior to stocks
test=true
Psize = rand(1:MaxSize,Nk)
DeliWindowsF=zeros(Nc,Nk,2)

while(test)
    global test=false
    global demands = rand(0:MaxDemand,Nc,Nk)
    global stocks = rand(0:MaxStock,Np,Nk)
    global DeliWindows = round.(Int, DeliWindowsF)
    totaldemand=zeros(Nk)
    maxdemand=zeros(Nk)
    for i in 1:Nc
        atleast1=0
        for j in 1:Nk
            if(demands[i,j]>0)
                DeliWindows[i,j,1]=rand(1:Nt)
                DeliWindows[i,j,2]=rand(DeliWindows[i,j,1]:Nt)
                atleast1=1
            end
            if(demands[i,j]>maxdemand[j])
                maxdemand[j]=demands[i,j]
            end
            totaldemand[j]+=demands[i,j]
        end
        if(atleast1==0)
            test=true
        end
    end
    totalsupply=zeros(Nk)
    for i in 1:Np
        atleast1=0
        for j in 1:Nk
            if(stocks[i,j]>0)
                atleast1=1
                if(stocks[i,j]<maxdemand[j])
                    stocks[i,j]=maxdemand[j]
                end
            end

            totalsupply[j]+=stocks[i,j]
        end
        if(atleast1==0 && i<=Np)
            test=true
        end
    end
    for i in 1:Nk
        if(totalsupply[i]< totaldemand[i])
            test=true
        end
    end
end
matrixDeliWindows = reshape(DeliWindows, Nc, Nk,2)


sizeP=Np+Nh
sizeC=Nc+Nh

test=true 
while(test)
    global Prod_av=rand(0:1, Np,Nt)
    c=1
    global test=false
    while(c <= Nc && !test)
        k=1
        while(k <= Nk && !test)
            if(demands[c,k]>0)
                if(DeliWindows[c,k,1] == DeliWindows[c,k,2] &&  DeliWindows[c,k,1]==1)
                    atleast1=false
                    for l in 1:Np
                        if(Prod_av[l,1]==1 && stocks[l,k]>=demands[c,k])
                            atleast1=true
                        end
                    end
                    if(!atleast1)
                        test=true
                    end
                end
            end
            k+=1
        end
        c+=1
    end
end
Client_av=rand(0:1,Nc,Nt)
Experiation_date=rand(1:Nt,Nk)
for c in 1:Nc
    global test=true
    while(test)
        k=1
        test=false
        Client_av[c,:]=rand(0:1,Nt)
        while(k <= Nk && !test)
            atleast1=false
            atleast1Prod=false
            if(demands[c,k]>0)
                for l in DeliWindows[c,k,1]:DeliWindows[c,k,2]
                    if(Client_av[c,l]==1)
                        atleast1=true
                    end
                end
                if(!atleast1)
                    test=true
                end
            end
            k+=1
        end
    end
end

# Create a JuMP model
model = Model(HiGHS.Optimizer)
set_silent(model)
##@variable(model, om[1:sizeC,1:Nt,1:Nv,1:TourVehicle[v],1:Nk,1:Nc], Bin);
##@variable(model, q[1:node,1:node,1:Nt,1:Nv,1:Nk,1:Nc], Bin);
@variable(model, 0<= u[i in 1:node,t in 1:Nt,v in 1:Nv,r in 1:TourVehicle[v]; StartVehicle[v]!=i]<=node, Int);
@variable(model, w[i in 1:node,t in 1:Nt,v in 1:Nv, r in 1:TourVehicle[v],k in 1:Nk ,c in 1:Nc], Bin);
@variable(model, f[i in 1:Np, k in 1:Nk, c in 1:Nc], Bin);
@variable(model, x[i in 1:node, j in 1:node, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v]; i!=j ], Bin)
@objective(model, Min, sum(dist2[i,j] * x[i,j,t,v,r] for i in 1:node, j in 1:node, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v] if j!=i )+sum(dist2[i,j+Np+Nh] * f[i, k, j]  for i in 1:Np, j in 1:Nc, k in 1:Nk));
## Each client is served exactly once
@constraint(model, client_service[j in 1:Nc, k in 1:Nk; demands[j,k]>0], sum(f[i, k, j] for i in 1:Np) == 1);
@constraint(model, stock_ok[i in 1:Np, k in 1:Nk], stocks[i,k] >= sum(f[i, k, j]*demands[j,k] for j in 1:Nc));
@constraint(model, Command_leave[i in 1:Np, k in 1:Nk,  c in 1:Nc], sum(w[i, t, v, r, k, c]*Prod_av[i,t] for t in 1:Nt for v in Vehicles[i] for r in 1:TourVehicle[v])+sum(w[i, t, v, r, k, c] for t in 1:Nt for h in 1:Nh for v in Vehicles[h+Np] for r in 1:TourVehicle[v])==f[i,k,c]);
#@constraint(model, Command_leave[i in 1:Np, k in 1:Nk,  c in 1:Nc], sum(w[i, t, v, r, k, c] for t in 1:Nt for v in 1:Nv for r in 1:TourVehicle[v])==f[i,k,c]);
@constraint(model, Cantleaveifunav[i in 1:Np, t in 1:Nt, v in Vehicles[i], k in 1:Nk, c in 1:Nc, r in 1:TourVehicle[v]], w[i, t, v, r, k, c] <= Prod_av[i,t]);

@constraint(model, Command_arrive[k in 1:Nk,  c in 1:Nc;demands[c,k]>0], sum(w[Cmoins[c], t, v, r, k, c]*Client_av[c,t] for t in DeliWindows[c,k,1]:DeliWindows[c,k,2] for v in 1:Nv for r in 1:TourVehicle[v])==1);
@constraint(model, Expi[i in 1:Np, k in 1:Nk,  c in 1:Nc, t in 1:Nt, tt in 1:Nt; tt<t || tt> t+Experiation_date[k] ], sum(w[i, t, v, r, k, c] for v in 1:Nv for r in 1:TourVehicle[v]) <= 1-sum(w[Cmoins[c], tt, v, r, k, c] for v in 1:Nv for r in 1:TourVehicle[v]));

@constraint(model, OneTour[i in Pplus, t in 1:Nt, v in Vehicles[i], r in 1:TourVehicle[v]], sum(x[i, j, t ,v, r] for j in 1:node if j!=i)<=1);
@constraint(model, flow[i in 1:node, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v]], sum(x[i, j, t ,v, r] for j in 1:node if j != i)==sum(x[j, i, t ,v, r] for j in 1:node if j != i));
@constraint(model, WorkingHour[t in 1:Nt, v in 1:Nv], sum(x[i, j, t ,v, r]*dist2[i,j] for j in 1:node for i in 1:node if i!=j for r in 1:TourVehicle[v])<=WorkVehicle[v]);

@constraint(model, wneedx[i in Pplus, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v], k in 1:Nk, c in 1:Nc], w[i, t, v, r, k, c] <= sum(x[i,j,t,v,r] for j in 1:node if i!=j));
@constraint(model, omneedx[i in Cmoins, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v], k in 1:Nk, c in 1:Nc], w[i, t, v, r, k, c] <= sum(x[j,i,t,v,r] for j in 1:node if i!=j));
@constraint(model, ProdDrop[i in Pplus, k in 1:Nk, c in 1:Nc, t in 1:Nt, v in Vehicles[i], r in 1:TourVehicle[v]], w[i, t, v, r, k, c] == w[Cmoins[c], t, v, r, k, c] + sum(w[PairHub[j,2], t, v, r, k, c] for j in 1:Nh if j+Np != i));
@constraint(model, CapacityOK[t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v]], sum(w[i, t, v, r, k, c] * demands[c, k] * Psize[k] for k in 1:Nk for c in 1:Nc for i in Pplus) <= CapaVehicle[v]);
#@constraint(model, Deliverythenpick[j in Cmoins, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v]; j!=StartVehicle[v]+Nh+Nc], sum(x[i, j, t ,v, r] for i in Pplus if StartVehicle[v]!=i)==0);

@constraint(model,DeliveryOrPick1[j in Cmoins, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v]; j!=StartVehicle[v]+Nh+Nc],sum(x[i, j, t ,v, r] for i in Pplus if StartVehicle[v]!=i)==0);
@constraint(model,DeliveryOrPick2[j in Cmoins, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v]],sum(x[j, i, t ,v, r] for i in Pplus if StartVehicle[v]!=i)==0);
##@constraint(model, ProdCantPick[i in 1:Np, v in Vehicles[i], t in 1:Nt, r in 1:TourVehicle[v]], sum(w[j, t ,v, r, k, c] for j in 1:Np if j != i for k in 1:Nk for c in 1:Nc)==0);
@constraint(model, delivernext[i in 1:Nh, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v], k in 1:Nk, c in 1:Nc], w[PairHub[i,1], t, v,r, k, c] <= sum(w[PairHub[i,2], tt, vv, rr, k, c] for tt in 1:t-1 for vv in 1:Nv for rr in 1:TourVehicle[vv]));
@constraint(model, HubDropAll[i in 1:Nh, k in 1:Nk, c in 1:Nc, t in 1:Nt, v in Vehicles[i+Np], r in 1:TourVehicle[v]], sum(w[j, t, v, r, k, c] for j in Pplus if PairHub[i,1]!=j) == w[PairHub[i,2], t, v, r, k, c]);
@constraint(model, flowhub[i in 1:Nh, v in Vehicles[i+Np], r in 1:TourVehicle[v], t in 1:Nt], sum(x[PairHub[i,1], j, t ,v, r] for j in 1:node if j != PairHub[i,1])==x[PairHub[i,2], PairHub[i,1], t ,v, r]);
@constraint(model, TooFarPoint[i in 1:Np, c in 1:Nc; dist2[i,c+Np+Nh] > WorkProd /2], sum(f[i,k,c]  for k in 1:Nk) == 0);

@constraint(model, subtour[v in 1:Nv, t in 1:Nt, r in 1:TourVehicle[v], i in 1:node, j in 1:node; i!=j && StartVehicle[v]!=i && StartVehicle[v]!=j], u[i,t,v,r]-u[j,t,v,r]+node*x[i,j,t,v,r]+(node-2)*x[j,i,t,v,r] <= node-1);
##@constraint(model, subtourqNp[t in 1:Nt, i in Pplus, k in 1:Nk, c in 1:Nc, v in 1:Nv], sum(q[j,i,t,v,k,c] for j in 1:node if j != i)+sum(w[i, t, v, r, k, c] for r in 1:TourVehicle[v])==sum(q[i,j,t,v,k,c] for j in 1:node if j != i));
##@constraint(model, subtourqNc[t in 1:Nt, i in Cmoins, k in 1:Nk, c in 1:Nc, v in 1:Nv], sum(q[j,i,t,v,k,c] for j in 1:node if j != i)==sum(q[i,j,t,v,k,c] for j in 1:node if j != i)+sum(w[i, t, v, r, k, c] for r in 1:TourVehicle[v]));
##@constraint(model, qneedx[i in 1:node, j in 1:node, t in 1:Nt, v in 1:Nv, k in 1:Nk, c in 1:Nc; i!=j], q[i, j, t, v, k, c] <= sum(x[i,j,t,v,r] for r in 1:TourVehicle[v]));
##@constraint(model, flowdepot[i in 1:Np, v in Vehicles[i], r in 1:TourVehicle[v], t in 1:Nt], sum(x[i, j, t ,v, r] for j in 1:node)==1);

##@constraint(model, WorkingHourProd0[t in 1:Nt, ii in 1:Np, v in Vehicles[ii]], sum(x[i, j, t ,v, r]*dist2[i,j] for j in 1:node for i in 1:node if i!=j for r in 1:TourVehicle[v])<=0);


    instance_name = "Full_$(Np)_$(Nc)_$(Nh)_$(se).lp"
file_path = "Full_$(Np)_$(Nc)_$(Nh)_$(se).data"
open(file_path, "w") do file
    # Write to the file
    write(file, "$Np $Nc $Nh $Nk $Nvh $CapaProd $WorkProd $CapaHub $WorkHub $Nt $TourHub\n")
    for i in 1:Np
        for j in 1:Nk
            write(file, "$(stocks[i,j]) ")
        end
    end
    write(file, "\n")
    for i in 1:Nc
        for j in 1:Nk
            write(file, "$(demands[i,j]) ")
        end
    end
    write(file, "\n")
    for i in 1:Np
        for j in 1:Nt
            write(file, "$(Prod_av[i,j]) ")
        end
    end
    write(file, "\n")
    for i in 1:Nc
        for j in 1:Nt
            write(file, "$(Client_av[i,j]) ")
        end
    end
    write(file, "\n")
    for i in 1:Nc
        for j in 1:Nk
            write(file, "$(DeliWindows[i,j,1]) $(DeliWindows[i,j,2]) ")
        end
    end
    write(file, "\n")
    for i in 1:Nk
        write(file, "$(Experiation_date[i]) ")
    end
    write(file, "\n")
    for i in 1:node
        for j in 1:node
            write(file, "$(dist2[i,j]) ")
        end
    end
    write(file, "\n")
    for i in 1:Nk
        write(file, "$(Psize[i]) ")
    end
    write(file, "\n")
    for i in 1:node
        write(file, "$(x_all[i]) $(y_all[i]) ")
    end
    
end
write_to_file(model,instance_name)

