
using JuMP
using CPLEX
# import HiGHS
# Import necessary libraries
using ArgParse
function solve_subproblem_bapcod(NbNodeSub, NodeSub,MaxTour,MaxWork,MaxCap,DemSub)
    open("/home/kisouke/bapcod-v0.82.5/Demos/VehicleRoutingWithTimeWindowsDemo/data/VRPTW/test.txt", "w") do file
        println(file,"CTest")
        println(file,"")
        println(file,"VEHICLE")
        println(file,"NUMBER \t CAPACITY")
        print(file,MaxTour)
        print(file,"\t")
        println(file,MaxCap)
        println(file,"")
        println(file,"CUSTOMER")
        println(file,"CUSTNO. \t XCOORD. \t YCOORD. \t DEMAND \t READYTIME \t DUEDATE \t SERVICE TIME")
        println(file,"")
        for i in 1:NbNodeSub
            print(file,i-1)
            print(file,"\t")
            print(file,x_all[NodeSub[i]])
            print(file,"\t")
            print(file,y_all[NodeSub[i]])
            print(file,"\t")
            print(file,DemSub[i])
            print(file,"\t")
            print(file,0)
            print(file,"\t")
            print(file,10000)
            print(file,"\t")
            println(file,0)
        end
    end
    command = `/home/kisouke/bapcod-v0.82.5/Demos/VehicleRoutingWithTimeWindowsDemo/bin/VehicleRoutingWithTimeWindowsDemo -b /home/kisouke/bapcod-v0.82.5/Demos/VehicleRoutingWithTimeWindowsDemo/config/bc.cfg -a /home/kisouke/bapcod-v0.82.5/Demos/VehicleRoutingWithTimeWindowsDemo/config/app.cfg -i /home/kisouke/bapcod-v0.82.5/Demos/VehicleRoutingWithTimeWindowsDemo/data/VRPTW/test.txt`

    # Run the command and capture the output
    output = read(command, String)

    # Filter the output to find the line starting with "Best found solution "
    for line in split(output, "\n")
        if startswith(line, "Best found solution ")
            println(line)
        end
    end
    #model = Model(HiGHS.Optimizer)
    #@variable(model, 0<= u[i in 1:NbNodeSub]<=NbNodeSub, Int);
    #@variable(model, x[i in 1:NbNodeSub, j in 1:NbNodeSub; i!=j ], Bin)
    #@constraint(model, flowout[j in 2:NbNodeSub], sum(x[i, j] for i in 1:NbNodeSub if j != i)==1);
    #@constraint(model, flowin[j in 2:NbNodeSub], sum(x[j, i] for i in 1:NbNodeSub if j != i)==1);
    #@constraint(model, flowdepot, sum(x[1, j] for j in 2:NbNodeSub)==sum(x[j, 1] for j in 2:NbNodeSub));
    #@constraint(model, Limvehicle, sum(x[1, j] for j in 2:NbNodeSub)<=MaxTour);
    #@constraint(model, WorkingHour, sum(x[i, j]*dist[NodeSub[i],NodeSub[j]] for j in 1:NbNodeSub for i in 1:NbNodeSub if i!=j)<=MaxWork);
    #@constraint(model, subtour[i in 2:NbNodeSub, j in 2:NbNodeSub; i!=j], u[i]-u[j]+NbNodeSub*x[i,j]+(NbNodeSub-2)*x[j,i] <= NbNodeSub-1);
    #@constraint(model, Capacity, sum(x[i, j]*dist[NodeSub[i],NodeSub[j]] for j in 1:NbNodeSub for i in 1:NbNodeSub if i!=j)<=MaxCap);

    #optimize!(model)
    #@assert is_solved_and_feasible(model; dual = true)
    return (0)
end

function solve_subproblem(NbNodeSub, NodeSub,MaxTour,MaxWork,MaxCap,DemSub)
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    @variable(model, 0<= u[i in 1:NbNodeSub, r in 1:MaxTour]<=NbNodeSub, Int);
    @variable(model, x[i in 1:NbNodeSub, j in 1:NbNodeSub,r in 1:MaxTour], Bin)
    @constraint(model, flowout[j in 2:NbNodeSub], sum(x[i, j, r] for i in 1:NbNodeSub for r in 1:MaxTour if i!=j)==1);
    @constraint(model, flowdepot[i in 1:NbNodeSub, r in 1:MaxTour], sum(x[i, j, r] for j in 1:NbNodeSub if i!=j)==sum(x[j, i, r] for j in 1:NbNodeSub if i!=j));
    @constraint(model, WorkingHour, sum(x[i, j, r]*dist[NodeSub[i],NodeSub[j]] for j in 1:NbNodeSub for i in 1:NbNodeSub for r in 1:MaxTour)<=MaxWork);
    @constraint(model, subtour[i in 2:NbNodeSub, j in 2:NbNodeSub,r in 1:MaxTour; i!=j], u[i,r]-u[j,r]+NbNodeSub*x[i,j,r]+(NbNodeSub-2)*x[j,i,r] <= NbNodeSub-1);
    @constraint(model, Capacity[r in 1:MaxTour], sum(x[i, j, r]*DemSub[j] for j in 1:NbNodeSub for i in 1:NbNodeSub if i!=j)<=MaxCap);


    @objective(model, Min, sum(dist[NodeSub[i],NodeSub[j]] * x[i, j, r]  for i in 1:NbNodeSub, j in 1:NbNodeSub, r in 1:MaxTour));
    optimize!(model)
    write_to_file(model, "work.lp")
    #println(x_is_selected)
    #println("Optimal value: ", objective_value(model))
    #for i in 1:NbNodeSub, j in 1:NbNodeSub, r in 1:MaxTour
    #    if x_is_selected[i, j, r]
    #            println("vehicle from $r travel from  $i to  $j")
    #    end
    #end
    status = termination_status(model)
    if status == MOI.OPTIMAL
        return objective_value(model)
    elseif status == MOI.INFEASIBLE
        return -1
    else
        println("Subproblem Solving failled")
    end

end

# Parse command-line arguments
s = ArgParseSettings()
@add_arg_table s begin
    "--instance"
    help = "Path to the first input file"
    "--ocut"
    help = "1 for Max and 2 for depot-point"
end
parsed_args = parse_args(s)

file_path1 = parsed_args["instance"]
ocut = parsed_args["ocut"]

# Initialize variables
count = 0

Experiation_date = []

Psize = []
x_all, y_all=[], []

# Read the file

open(file_path1, "r") do file
    count = 0
    print("read ")
    println(file_path1)
    for line in eachline(file)
        elements = split(strip(line))
        if count == 0
            global Np = parse(Int, elements[1])
            global Nc = parse(Int, elements[2])
            global Nh = parse(Int, elements[3])
            global Nk = parse(Int, elements[4])
            global Nvh = parse(Int, elements[5])
            global CapaProd = parse(Int, elements[6])
            global WorkProd = parse(Int, elements[7])
            global CapaHub = parse(Int, elements[8])
            global WorkHub = parse(Int, elements[9])
            global Nt = parse(Int, elements[10])
            global TourHub = parse(Int, elements[11])
            global node = Np + Nc + 2 * Nh
            global Nv = Nvh * Nh + Np
        elseif count == 1
            count2 = 1
            global stocks = zeros(Np, Nk)
            for i in 1:Np
                for j in 1:Nk
                    stocks[i,j]=parse(Int, elements[count2])
                    count2 += 1
                end
            end
        elseif count == 2
            count2 = 1
            global demands = zeros(Nc, Nk)
            for i in 1:Nc
                for j in 1:Nk
                    demands[i,j]= parse(Int, elements[count2])
                    count2 += 1
                end
            end
        elseif count == 3
            count2 = 1
            global Prod_av = zeros(Np, Nt)
            for i in 1:Np
                for j in 1:Nt
                    Prod_av[i,j]=parse(Int, elements[count2])
                    count2 += 1
                end
            end
        elseif count == 4
            count2 = 1
            global Client_av = zeros(Nc, Nt)
            for i in 1:Nc
                for j in 1:Nt
                    Client_av[i,j]=parse(Int, elements[count2])
                    count2 += 1
                end
            end
        elseif count == 5
            count2 = 1
            global DeliWindowsF=zeros(Nc,Nk,2)
            global DeliWindows = round.(Int, DeliWindowsF)
            for i in 1:Nc
                for j in 1:Nk
                    DeliWindows[i,j,1]=parse(Int, elements[count2])
                    count2 += 1
                    DeliWindows[i,j,2]=parse(Int, elements[count2])
                    count2 += 1
                end
            end
        elseif count == 6
            for i in 1:Nk
                push!(Experiation_date, parse(Int, elements[i]))
            end
        elseif count == 7
            count2 = 1
            global dist = zeros(node, node)
            global MinDist=zeros(node)
            for i in 1:node
                MinD=10000000
                for j in 1:node
                    if i!=j && MinD > dist[i,j]
                        MinD=dist[i,j]
                    end
                    dist[i,j]= parse(Float64, elements[count2])
                    count2 += 1
                end
                @assert(10000000>MinD)
                MinDist[i]=MinD
            end
        elseif count == 8
            for i in 1:Nk
                push!(Psize, parse(Int, elements[i]))
            end
        elseif count == 9
            count2=1
            for i in 1:Np+Nc+Nh+Nh
                push!(x_all, parse(Int, elements[count2]))
                count2 +=1
                push!(y_all, parse(Int, elements[count2]))
                count2 +=1
            end
        end
        count += 1
    end
end
## Clients' locations
x_c, y_c = zeros(Nc), zeros(Nc)

## Producer' potential locations
x_p, y_p = zeros(Np), zeros(Np)

## Hub' potential locations
x_h, y_h = zeros(Nh), zeros(Nh)

ClientsF=zeros(Nh+Np)
Clients = round.(Int, ClientsF)
Vehicles=[]
StartVehicle=Int[]
CapaVehicle=Int[]
WorkVehicle=Int[]
TourVehicle=Int[]
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
sizeP=Np+Nh
sizeC=Nc+Nh
println("Model the problem")
MAXIMUM_ITERATIONS = 1000
ABSOLUTE_OPTIMALITY_GAP = 1e-6
# Create a JuMP model
model = Model(CPLEX.Optimizer)
set_silent(model)
##@variable(model, om[1:sizeC,1:Nt,1:Nv,1:TourVehicle[v],1:Nk,1:Nc], Bin);
##@variable(model, q[1:node,1:node,1:Nt,1:Nv,1:Nk,1:Nc], Bin);
@variable(model, w[i in 1:node,t in 1:Nt,v in 1:Nv,k in 1:Nk ,c in 1:Nc], Bin);
@variable(model, f[i in 1:Np, k in 1:Nk, c in 1:Nc], Bin);
#@variable(model, y[i in 1:node, t in 1:Nt, v in 1:Nv], Bin);
@variable(model,0.0<=sigma[t in 1:Nt, v in 1:Nv]);
@objective(model, Min, sum(dist[i,j+Np+Nh] * f[i, k, j]  for i in 1:Np, j in 1:Nc, k in 1:Nk)+sum(sigma[t,v] for t in 1:Nt, v in 1:Nv));
## Each client is served exactly once
@constraint(model, client_service[j in 1:Nc, k in 1:Nk; demands[j,k]>0], sum(f[i, k, j] for i in 1:Np) == 1);
@constraint(model, stock_ok[i in 1:Np, k in 1:Nk], stocks[i,k] >= sum(f[i, k, j]*demands[j,k] for j in 1:Nc));
@constraint(model, Command_leave[i in 1:Np, k in 1:Nk,  c in 1:Nc], sum(w[i, t, v, k, c]*Prod_av[i,t] for t in 1:Nt for v in Vehicles[i])+sum(w[i, t, v, k, c] for t in 1:Nt for h in 1:Nh for v in Vehicles[h+Np])==f[i,k,c]);
@constraint(model, Cantleaveifunav[i in 1:Np, t in 1:Nt, v in Vehicles[i], k in 1:Nk, c in 1:Nc], w[i, t, v, k, c] <= Prod_av[i,t]);

@constraint(model, Command_arrive[k in 1:Nk,  c in 1:Nc;demands[c,k]>0], sum(w[Cmoins[c], t, v, k, c]*Client_av[c,t] for t in DeliWindows[c,k,1]:DeliWindows[c,k,2] for v in 1:Nv)==1);
@constraint(model, Expi[i in 1:Np, k in 1:Nk,  c in 1:Nc, t in 1:Nt, tt in 1:Nt; tt<t || tt> t+Experiation_date[k] ], sum(w[i, t, v, k, c] for v in 1:Nv) <= 1-sum(w[Cmoins[c], tt, v, k, c] for v in 1:Nv));

@constraint(model, ProdDrop[i in Pplus, k in 1:Nk, c in 1:Nc, t in 1:Nt, v in Vehicles[i]], w[i, t, v, k, c] == w[Cmoins[c], t, v, k, c] + sum(w[PairHub[j,2], t, v, k, c] for j in 1:Nh if j+Np != i));
@constraint(model, HubDropAll[i in 1:Nh, k in 1:Nk, c in 1:Nc, t in 1:Nt, v in Vehicles[i+Np]], sum(w[j, t, v, k, c] for j in Pplus if PairHub[i,1]!=j) == w[PairHub[i,2], t, v, k, c]);

@constraint(model, delivernext[i in 1:Nh, t in 1:Nt, v in 1:Nv, k in 1:Nk, c in 1:Nc], w[PairHub[i,1], t, v, k, c] <= sum(w[PairHub[i,2], tt, vv, k, c] for tt in 1:t-1 for vv in 1:Nv));


@constraint(model, CapacityOK[t in 1:Nt, i in 1:Np, v in Vehicles[i]], sum(w[i, t, v, k, c] * demands[c, k] * Psize[k] for k in 1:Nk for c in 1:Nc) <= CapaVehicle[v]);
#@constraint(model, CapacityOK[t in 1:Nt, i in Pplus, v in Vehicles[i]], sum(w[i, t, v, k, c] * demands[c, k] * Psize[k] for k in 1:Nk for c in 1:Nc) <= CapaVehicle[v]*TourVehicle[v]);

@constraint(model, ProdCantPick[t in 1:Nt, i in 1:Np, v in Vehicles[i]], sum(w[j, t, v, k, c] for j in Pplus for k in 1:Nk for c in 1:Nc if j!=i) == 0);

#@constraint(model, TooFarPoint[i in Pplus, v in Vehicles[i]], sum(w[j, t, v, k, c]  for t in 1:Nt for k in 1:Nk for c in 1:Nc for j in 1:node if dist[i,j] > (WorkVehicle[v]/2) ) == 0);
@constraint(model, TooFarPoint[i in 1:Np, c in 1:Nc], sum(f[i,k,c]  for k in 1:Nk if dist[i,c+Np+Nh] > (WorkVehicle[v]/2) ) == 0);

#@constraint(model, MaxSigma[t in 1:Nt, v in 1:Nv], sigma[t,v] <= WorkVehicle[v])

#@constraint(model,tctr[i in 1:node, t in 1:Nt, v in 1:Nv, k in 1:Nk, c in 1:Nc], y[i,t,v] >= w[i,t,v,k,c])
#@constraint(model, TourEval[i in Pplus, v in Vehicles[i], t in 1:Nt], sum(y[j,t,v] * 2* MinDist[j] for j in 1:node if j!=i)<= WorkVehicle[v])
#@variable(model, 0<= u[i in 1:node,t in 1:Nt,v in 1:Nv,r in 1:TourVehicle[v]; StartVehicle[v]!=i]<=node, Int);
#@variable(model, x[i in 1:node, j in 1:node, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v]; i!=j ], Bin)

#@constraint(model, flow[i in 1:node, t in 1:Nt, v in 1:Nv], sum(x[i, j, t ,v, r] for j in 1:node if j != i)==sum(x[j, i, t ,v, r] for j in 1:node if j != i));
#@constraint(model, WorkingHour[t in 1:Nt, v in 1:Nv], sum(x[i, j, t ,v, r]*dist[i,j] for j in 1:node for i in 1:node if i!=j for r in 1:TourVehicle[v])<=WorkVehicle[v]);
#@constraint(model, subtour[v in 1:Nv, t in 1:Nt, r in 1:TourVehicle[v], i in 1:node, j in 1:node; i!=j && StartVehicle[v]!=i && StartVehicle[v]!=j], u[i,t,v,r]-u[j,t,v,r]+node*x[i,j,t,v,r]+(node-2)*x[j,i,t,v,r] <= node-1);
#@constraint(model, flowhub[i in 1:Nh, v in Vehicles[i+Np], r in 1:TourVehicle[v], t in 1:Nt], sum(x[PairHub[i,1], j, t ,v, r] for j in 1:node if j != PairHub[i,1])==x[PairHub[i,2], PairHub[i,1], t ,v, r]);

#@constraint(model, wneedx[i in Pplus, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v], k in 1:Nk, c in 1:Nc], w[i, t, v, r, k, c] <= sum(x[i,j,t,v,r] for j in 1:node if i!=j));
#@constraint(model, omneedx[i in Cmoins, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v], k in 1:Nk, c in 1:Nc], w[i, t, v, r, k, c] <= sum(x[j,i,t,v,r] for j in 1:node if i!=j));
#@constraint(model, Deliverythenpick[j in Cmoins, t in 1:Nt, v in 1:Nv, r in 1:TourVehicle[v]; j!=StartVehicle[v]+Nh+Nc], sum(x[i, j, t ,v, r] for i in Pplus if StartVehicle[v]!=i)==0);
#@constraint(model, OneTour[i in Pplus, t in 1:Nt, v in Vehicles[i], r in 1:TourVehicle[v]], sum(x[i, j, t ,v, r] for j in 1:node if j!=i)<=1);
SigmaVar=[]
ite = 0
total_time = 0.0
time_solve_sub = 0.0
time_solve_master=0.0
time_gen_sub=0.0
time_gen_cub=0.0
lower = 0
upper = 0
gap=0
nbOpt=0
nbFeas=0
println(sum(num_constraints(model, F, S) for (F, S) in list_of_constraint_types(model)))
println("Number of variables: ", num_variables(model))
write_to_file(model, "model.lp")

println("Solve")
Bestupper=10000
global total_time = @elapsed begin
    while ite < MAXIMUM_ITERATIONS
        #println("NEW ITERATION +------------------------------------------------------------")
        global ite = ite + 1
        #print("BENDER ITERATION NUBMER ")
        #println(ite)
        elapsed_time = @elapsed begin
            optimize!(model)
        end
        status = termination_status(model)
        if status == MOI.INFEASIBLE
            println("Problem is unfeasible")
            break
        end
        global time_solve_master += elapsed_time

        global lower = objective_value(model)
        w_is_selected = isapprox.(value.(w), 1; atol = 1e-5);
        global f_is_selected = isapprox.(value.(f), 1; atol = 1e-5);
        global upper = sum(dist[i,j+Np+Nh] * f_is_selected[i, k, j]  for i in 1:Np, j in 1:Nc, k in 1:Nk)
        #=TestOptSol= dist[1,5] + dist[1,8]+ dist[2,4] + dist[2,6] + dist[2,6] + dist[2,7] + dist[2,7] + dist[2,8] + dist[2,5] + value.(sigma[2,1]) +  value.(sigma[4,2]) + value.(sigma[3,2]) + value.(sigma[4,3]) + value.(sigma[5,4]) +  value.(sigma[1,3]) + value.(sigma[2,3]) + value.(sigma[2,4]) + value.(sigma[3,4])
        print("OPTVALUE ")
        println(TestOptSol)
        print("Initial UP ")
        println(upper)
        print("Initial low ")
        println(lower)=#
        sigmaval= value.(sigma);
        #=println(sigmaval)
        for i in 1:Np, j in 1:Nc, k in 1:Nk
            print(dist[i,j]*f_is_selected[i, k, j])
            print(" ")
        end=#
        for t in 1:Nt, v in 1:Nv
            #=print("Solve sub problem ")
            print(t)
            print(" ")
            println(v)=#

            if StartVehicle[v]>Np || Prod_av[StartVehicle[v],t]==1
                elapsed_time = @elapsed begin
                    NbNodeSub=[1,1]
                    NodeSub=[[StartVehicle[v]],[StartVehicle[v]]]
                    DemandSub=[[0],[0]]
                    DistDepPoint=[[],[]]
                    DistMax=[[],[]]
                    WSub=[[],[]]
                    for k in 1:Nk, i in 1:node, c in 1:Nc
                        if w_is_selected[i, t, v, k, c] && StartVehicle[v]!=i
                            if i in Pplus
                                push!(NodeSub[1],i)
                                push!(DemandSub[1],demands[c, k] * Psize[k])
                                push!(WSub[1],(i,k,c))
                                push!(DistDepPoint[1],2*dist[StartVehicle[v],i])
                                NbNodeSub[1]+=1
                            else
                                push!(NodeSub[2],i)
                                push!(DemandSub[2],demands[c, k] * Psize[k])
                                push!(WSub[2],(i,k,c))
                                push!(DistDepPoint[2],2*dist[StartVehicle[v],i])
                                NbNodeSub[2]+=1
                            end

                        end
                    end
                end
                global time_gen_sub += elapsed_time
                ret=[0.0,0.0]
                for s in 1:2
                    if NbNodeSub[s]>1
                        elapsed_time = @elapsed begin
                            ret[s] = solve_subproblem(NbNodeSub[s], NodeSub[s],TourVehicle[v],WorkVehicle[v],CapaVehicle[v],DemandSub[s])
                        end
                        global time_solve_sub += elapsed_time
                        global upper+=ret[s]
                        elapsed_time = @elapsed begin
                            if ret[s] ==-1
                                #print("NEW SUBPr --------------------------------------------------------")
                                #println(WSub)
                                upper+=1000
                                #=for (l, (i, k, c)) in enumerate(WSub)
                                    print(l)
                                    print(" ")
                                    print(i)
                                    print(" ")
                                    print(k)
                                    print(" ")
                                    println(c)
                                end
                                print(pp)
                                print("fails for vehicule ")
                                print(v)
                                print(" with start ")
                                print(StartVehicle[v])
                                print(" nb node ")
                                print(NbNodeSub)
                                print(" and drop cap ")
                                print(DropCap)
                                print(" and pick cap ")
                                println(PickCap)=#
                                #cut=@constraint(model, sum(w[i, t, v, k, c] for (l, (i, k, c)) in enumerate(WSub)) <= NbNodeSub-2);
                                for tt in 1:Nt, vv in Vehicles[StartVehicle[v]]
                                    if StartVehicle[vv]>Np || Prod_av[StartVehicle[vv],tt]==1
                                        cut=@constraint(model, sum(w[i, tt, vv, k, c] for (l, (i, k, c)) in enumerate(WSub[s])) <= NbNodeSub[s]-2);
                                        global nbFeas+=1
                                    end
                                    #@info "Adding the feasability cut $(cut)"
                                    #=for (l, (i, k, c)) in enumerate(WSub)
                                        TempWSub=WSub
                                        for kk in 1:Nk
                                            if demands[c, k] * Psize[k] <= demands[c, kk] * Psize[kk] && ((i,kk,c) in WSub)==false
                                                TempWSub[l]=(i,kk,c)
                                                #println(TempWSub)
                                                cut=@constraint(model, sum(w[ii, tt, vv, kkk, cc] for (l, (ii, kkk, cc)) in enumerate(TempWSub)) <= NbNodeSub-2);
                                                #@info "Adding the feasability cut $(cut)"
                                                global nbFeas+=1
                                            end
                                        end
                                    end=#
                                end
                                #@info "Adding the feasability cut"
                                empty!(WSub[s])
                                ret[s]=0
                            end
                        end
                    end
                    global time_gen_cub += elapsed_time
                end
                elapsed_time = @elapsed begin
                    if ret[1]+ret[2] > sigmaval[t,v] + 0.00001
                        #=print(ret)
                        print(" ")
                        println(sigmaval[t,v])
                        @assert(ret+0.1>=sigmaval[t,v])=#

                        #println(WSub)
                        if ocut=="1"
                            for i in 2:NbNodeSub[1]
                                Max=0
                                for j in 1:NbNodeSub[1]
                                    if Max < dist[NodeSub[1][i],NodeSub[1][j]]
                                        Max=dist[NodeSub[1][i],NodeSub[1][j]]
                                    end
                                end
                                push!(DistMax[1],2*Max)
                            end
                            for i in 2:NbNodeSub[2]
                                Max=0
                                for j in 1:NbNodeSub[2]
                                    if Max < dist[NodeSub[2][i],NodeSub[2][j]]
                                        Max=dist[NodeSub[2][i],NodeSub[2][j]]
                                    end
                                end
                                push!(DistMax[2],2*Max)
                            end
                        end
                        #=if ocut=="1"
                            cut=@constraint(model, ret[1] +ret[2] - sum(DistMax[1][l] * (1-w[i, t, v, k, c]) for (l, (i, k, c)) in enumerate(WSub[1])) -sum(DistMax[2][l] * (1-w[i, t, v, k, c]) for (l, (i, k, c)) in enumerate(WSub[2])) <= sigma[t,v]);
                        else
                            cut=@constraint(model, ret[1] +ret[2] - sum(DistDepPoint[1][l] * (1-w[i, t, v, k, c]) for (l, (i, k, c)) in enumerate(WSub[1])) -sum(DistDepPoint[2][l] * (1-w[i, t, v, k, c]) for (l, (i, k, c)) in enumerate(WSub[2])) <= sigma[t,v]);
                        end=#
                        #@info "Adding the optimality cut " cut
                        #print(DistMax)
                        for tt in 1:Nt, vv in Vehicles[StartVehicle[v]]
                            if StartVehicle[v]>Np || Prod_av[StartVehicle[v],t]==1
                                init=0
                                for m in DistDepPoint[2]
                                    init-=m
                                end
                                for m in DistDepPoint[1]
                                    init-=m
                                end
                                init+=ret[1] +ret[2]
                                if ocut=="1"
                                    cut=@constraint(model, round(init,digits=2) - sum(DistMax[1][l] * (-w[i, tt, vv, k, c]) for (l, (i, k, c)) in enumerate(WSub[1])) -sum(DistMax[2][l] * (-w[i, tt, vv, k, c]) for (l, (i, k, c)) in enumerate(WSub[2]))  <= sigma[tt,vv]);
                                else
                                    cut=@constraint(model,  round(init,digits=2) - sum(DistDepPoint[1][l] * (-w[i, tt, vv, k, c]) for (l, (i, k, c)) in enumerate(WSub[1])) - sum(DistDepPoint[2][l] * (-w[i, tt, vv, k, c]) for (l, (i, k, c)) in enumerate(WSub[2])) <= sigma[tt,vv]);
                                end
                                #@info "Adding the optimality cut " cut
                                global nbOpt+=1
                            end
                            #=for (l, (i, k, c)) in enumerate(WSub)
                                TempWSub=WSub
                                for kk in 1:Nk
                                    if demands[c, k] * Psize[k] <= demands[c, kk] * Psize[kk] && ((i,kk,c) in WSub)==false
                                        TempWSub[l]=(i,kk,c)
                                        #println(TempWSub)
                                        if ocut=="1"
                                            cut=@constraint(model, ret - sum(DistMax[ll] * (1-w[ii, tt, vv, kkk, cc]) for (ll, (ii, kkk, cc)) in enumerate(TempWSub)) <= sigma[tt,vv]);
                                        else
                                            cut=@constraint(model, ret - sum(DistDepPoint[ll] * (1-w[ii, tt, vv, kkk, cc]) for (ll, (ii, kkk, cc)) in enumerate(TempWSub)) <= sigma[tt,vv]);
                                        end
                                        #@info "Adding the feasability cut $(cut)"
                                        global nbOpt+=1
                                    end
                                end
                            end=#
                        end

                        #@info "Adding the optimality cut "
                    end

                    if ret[1]+ret[2]> WorkVehicle[v]
                        upper+=1000
                        for tt in 1:Nt, vv in Vehicles[StartVehicle[v]]
                            if StartVehicle[v]>Np || Prod_av[StartVehicle[v],t]==1
                                cut=@constraint(model, sum(w[i, tt, vv, k, c] for (l, (i, k, c)) in enumerate(WSub[1]))+sum(w[i, tt, vv, k, c] for (l, (i, k, c)) in enumerate(WSub[2])) <= NbNodeSub[1]+NbNodeSub[2]-3);
                                global nbFeas+=1
                            end
                            #@info "Adding the feasability cut $(cut)"
                            #=for (l, (i, k, c)) in enumerate(WSub)
                                TempWSub=WSub
                                for kk in 1:Nk
                                    if demands[c, k] * Psize[k] <= demands[c, kk] * Psize[kk] && ((i,kk,c) in WSub)==false
                                        TempWSub[l]=(i,kk,c)
                                        #println(TempWSub)
                                        cut=@constraint(model, sum(w[ii, tt, vv, kkk, cc] for (l, (ii, kkk, cc)) in enumerate(TempWSub)) <= NbNodeSub-2);
                                        #@info "Adding the feasability cut $(cut)"
                                        global nbFeas+=1
                                    end
                                end
                            end=#
                            #@info "Adding the feasability cut" cut
                        end

                    end
                #else
                    #println(sigmaval[t,v])
                    #@assert(0.1>=sigmaval[t,v])
                end
                global time_gen_cub += elapsed_time
            end
        end
        if upper < Bestupper
            global Bestupper=upper
        end
        global gap = (upper - lower) / upper
        #print("UPPER BOUND ")
        #print(upper)
        #print(" Lower bounds ")
        #println(lower)
        #println(gap)
        print(ite)
        print(" ")
        print(upper)
        print(" ")
        print(lower)
        print(" ")
        print(gap)
        print(" Feas ")
        print(nbFeas)
        print(" Opt ")
        println(nbOpt)
        if time_solve_master+ time_solve_sub > 1800
            println("Time limit reached")
            break
        end
        if gap < ABSOLUTE_OPTIMALITY_GAP
            println("Terminating with the optimal solution")
            println("Optimal value: ", lower)
            break
        end

    end
end


print("Iteration: ")
println(ite)
print("Upper: ")
println(Bestupper)
print("Lower: ")
println(lower)
print("Gap: ")
println(gap)
print("Total time: ")
println(total_time)
print("Time to solve Masters: ")
println(time_solve_master)
print("Time to solve Subs: ")
println(time_solve_sub)
print("Time to gen Subs: ")
println(time_gen_sub)
print("Time to gen Cuts: ")
println(time_gen_cub)
print("Nb Feas: ")
println(nbFeas)
print("Nb Opt: ")
println(nbOpt)
optimize!(model)

#=w_is_selected = isapprox.(value.(w), 1; atol = 1e-5);
f_is_selected = isapprox.(value.(f), 1; atol = 1e-5);
sigma_is_selected = value.(sigma);

for c in 1:Nc, k in 1:Nk
    if demands[c,k]>0
        print(demands)
        print("Command ")
        print(c)
        print(" ")
        print(k)
        for i in 1:Np
            if f_is_selected[i, k, c]
                print(" is taken by ")
                println(i)
            end
        end
        print("Follow Route: ")
        for i in 1:node, v in 1:Nv, t in 1:Nt
            if w_is_selected[i, t, v, k, c]
                print("node ")
                print(i)
                print(" vehicule ")
                print(v)
                print(" period ")
                println(t)
            end
        end
    end
end
for t in 1:Nt, v in 1:Nv
    print(t)
    print(" ")
    print(v)
    print(" ")
    println(sigma_is_selected[t,v])
end =#
