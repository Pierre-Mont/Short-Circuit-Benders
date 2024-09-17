#!/bin/bash
 # Get the current directory
        directory=$1

        option=$2

        # Change to the directory containing the .lp files (if not already in the correct directory)
        cd "$directory" || { echo "Failed to change directory to $directory"; exit 1; }
        echo ";Status;Iteration;Upper;Lower;Total time;Time to solve Masters;Time to solve Subs;Nb Feas cut; Nb Opt cut ; Average nodes in subs; Min nodes in subs; Max nodes in subs; Delivery Prod; Delivery Hub" >> Sol$option.csv
        # Loop over each .lp file in the current directory
        for inst in *.data; do
                    echo "Solving $inst"
                        echo -n $inst >> Sol$option.csv
                            # Solve the .lp file with CPLEX and store the result in the .sol file
                            ../../build/Bender  $inst $option | awk '
                                  /Optimal value/ {optimal_value=$NF}
                                    /Iteration/ {iteration=$NF}
                                      /Upper/ {upper=$NF}
                                        /Lower/ {lower=$NF}
                                            /Total time/ {total_time=$NF}
                                              /Master Solving/ {time_masters=$NF}
                                                /Sub Solving/ {time_subs=$NF}
                                                      /NbFeas/{NbFeas=$NF}
                                                        /NbOpt/{NbOpt=$NF}
                                                          /Terminating with the optimal solution/ {Status="OPT"}
                                                            /Problem is unfeasible/ {Status="UNF"}
                                                            /Min Node Subs/ {MinN=$NF}
                                                            /Max Node Subs/ {MaxN=$NF}
                                                            /Average Node Subs/ {AVN=$NF}
                                                            /Delivery Prod/ {DP=$NF}
                                                            /Delivery Hub/ {DH=$NF}
                                                              END {
                                                                  print ";" Status ";" iteration ";" upper " ;" lower " ;" total_time ";" time_masters " ; " time_subs " ;" NbFeas ";" NbOpt ";" AVN ";" MinN ";" MaxN ";" DP ";" DH} ' >> Sol$option.csv
                                                          done
