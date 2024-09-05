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
    
    for ((i=1; i<=num; i++)); do
        # Create the file path
        filePath="Inst_${in1}_${in2}_${in3}_${i}.txt"
        echo "Generating instance $i with inputs $in1 and $in2"
        
        # Create the content of the file
        line="## Seed NbPériode MaxCoordonnée NbProduit MaxSize\n$i 5 20 2 10\n## NbProd Capacité-Véhicule-Prod Working-Hour-Prod MaxStock\n$in1 40 30 10\n## NbClient MaxQuantité_Prod\n$in2 3\n## NbHub NbVéhicule Capacité Wroking-Hour TourHub\n$in3 2 60 60 3"
        
        # Write the content to the file
        echo -e "$line" > "$filePath"
        
        # Run the Julia command
        julia "$Generator" "$filePath"
        rm $filePath
    done
}

# Call the function with the provided inputs
generate_instances "$numInstances" "$input1" "$input2" "$input3"
