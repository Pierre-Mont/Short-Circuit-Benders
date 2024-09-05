#!/bin/bash
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <directory>"
     exit 1
        fi

        # Get the current directory
        directory=$1

        # Change to the directory containing the .lp files (if not already in the correct directory)
        cd "$directory" || { echo "Failed to change directory to $directory"; exit 1; }
        echo "Status;Iteration;Upper;Lower;Gap;Total time;Time to solve Masters;Time to solve Subs;Time to gen Subs;Time to gen Cuts;Nb Feas cut; Nb Opt cut" >> Sol.csv
        # Loop over each .lp file in the current directory
        for inst in *.data; do
                    echo "Solving $inst"
                        echo -n $inst >> Sol.csv
                            # Solve the .lp file with CPLEX and store the result in the .sol file
                            time /home/kisouke/BenderCplex/build/Bender  $inst | awk '
                                  /Optimal value/ {optimal_value=$NF}
                                    /Iteration/ {iteration=$NF}
                                      /Upper/ {upper=$NF}
                                        /Lower/ {lower=$NF}
                                            /user/ {total_time=$NF}
                                              /Master Solving/ {time_masters=$NF}
                                                /Sub Solving/ {time_subs=$NF}
                                                      /NbFeas/{NbFeas=$NF}
                                                        /NbOpt/{NbOpt=$NF}
                                                          /Terminating with the optimal solution/ {Status="OPT"}
                                                            /Problem is unfeasible/ {Status="UNF"}
                                                              END {
                                                                  print ";" Status ";" iteration ";" upper " ;" lower " ;" total_time ";" time_masters " ; " time_subs " ;" NbFeas ";" NbOpt} ' >> Sol.csv
                                                          done
