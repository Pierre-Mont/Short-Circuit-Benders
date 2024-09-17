#!/bin/bash

# Define the parameters
Generator=$1
numInstances=$2
input1=$3
input2=$4
input3=$5


# Function to generate instances
generate_instances() {
    local num=$1
    local in1=$2
    local in2=$3
    local in3=$4
    i=1
    seed=1
    while [ $i -le $num ]; do
        # Create the file path
        filePath="Inst_${in1}_${in2}_${in3}_${seed}.txt"
        echo "Generating instance $i with inputs $in1 and $in2"
        
        # Create the content of the file
        line="## Seed NbPériode MaxCoordonnée NbProduit MaxSize\n$seed 3 20 2 10\n## NbProd Capacité-Véhicule-Prod Working-Hour-Prod MaxStock\n$in1 40 50 15\n## NbClient MaxQuantité_Prod\n$in2 3\n## NbHub NbVéhicule Capacité Wroking-Hour TourHub\n$in3 2 60 60 3"
        
        # Write the content to the file
        echo -e "$line" > "$filePath"
        echo $filePath
        filePathlp="Inst_${in1}_${in2}_${in3}_${seed}.lp"
        # Run the Julia command
        julia "$Generator" "$filePath"
        # Solve the .lp file with CPLEX and store the result in the .sol file
        result=$(/opt/ibm/ILOG/CPLEX_Studio2211/cplex/bin/x86-64_linux/cplex -c "
        read $filePathlp
        set timelimit 5
        optimize
        quit
        " 2>&1 | awk '
            BEGIN {
                Statut = "Unknown"
                opt = "Not Set"
            }
            /Objective/ {
                split($0, parts, " ")
                opt = parts[length(parts)]
            }
            /integer feasible:q/ {
                Statut = "Feas"
            }
            /Integer optimal/ {
                Statut = "Opt"
            }
            /Integer infeasible/ {
                opt = -2
                Statut = "Inf"
            }
            /Solution time/ {
                split($0, parts, " ")
                Time = parts[4]
            }
            END {
                print Statut
            }
        ')

        echo "Result from CPLEX: $result"

        if [ "$result" != "Inf" ]; then
            echo "Iteration $i:  solution found."
            ((i++))
        else
            
            filePathdata="Inst_${in1}_${in2}_${in3}_${seed}.data"
            rm $filePathdata
            rm $filePathlp
        fi
        ((seed++))
        rm $filePath
    done
}

# Call the function with the provided inputs
generate_instances "$numInstances" "$input1" "$input2" "$input3"
rm *.log