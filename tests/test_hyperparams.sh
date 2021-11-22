#!/bin/bash

memory=1
max_horizon=10
worlds=('mabc' 'recycling' 'tiger' 'alignment_2x4' 'boxPushingUAI07' 'GridSmall' 'Mars')
 
for world in "${worlds[@]}"
do
    for (( h=1; h<=$max_horizon; h++ ))
    do 
        echo "Run HSVI, $world, h=$h, please wait ..."
        SDMStudio solve -a HSVI -f oMDP -h $h -m $memory -e 0.01 --p_c 0.1 --p_o 0.001 --p_b 0.01 --lower_bound maxplan_wcsp --lb_freq_pruning 10 --lb_type_of_pruning bounded --upper_bound sawtooth_lp --ub_init Pomdp --time_max 7200 -w "${world}.dpomdp" --name "hsvi_${world}_${h}_${memory}_wcsp_lp"
        echo "Run sequential HSVI, world=$world, h=$h, please wait ..."
        SDMStudio solve -a HSVI -f ext-oMDP -h $h -m $memory -e 0.01 --p_c 0.1 --p_o 0.001 --p_b 0.01 --lower_bound maxplan --lb_freq_pruning 10 --lb_type_of_pruning bounded --upper_bound sawtooth_lp --ub_init Pomdp --time_max 7200 -w "${world}.dpomdp" --name "ext-hsvi_${world}_${h}_${memory}_wcsp_lp"
        echo "Run PBVI, world=$world, h=$h, please wait ..."
        SDMStudio solve -a PBVI -f oMDP -h $h -m $memory -e 0.01 --p_c 0.1 --p_o 0.001 --p_b 0.01 --lower_bound maxplan_wcsp --time_max 7200 --num_samples 100 -w "${world}.dpomdp" --name "pbvi100_${world}_${h}_${memory}_wcsp"
        echo "Run sequential PBVI, world=$world, h=$h, please wait ..."
        SDMStudio solve -a PBVI -f ext-oMDP -h $h -m $memory -e 0.01 --p_c 0.1 --p_o 0.001 --p_b 0.01 --lower_bound maxplan --time_max 7200 --num_samples 100 -w "${world}.dpomdp" --name "ext-pbvi100_${world}_${h}_${memory}_wcsp"
        echo "Run PBVI, world=$world, h=$h, please wait ..."
        SDMStudio solve -a PBVI -f oMDP -h $h -m $memory -e 0.01 --p_c 0.1 --p_o 0.001 --p_b 0.01 --lower_bound maxplan_wcsp --time_max 7200 --num_samples 1000 -w "${world}.dpomdp" --name "pbvi1000_${world}_${h}_${memory}_wcsp"
        echo "Run sequential PBVI, world=$world, h=$h, please wait ..."
        SDMStudio solve -a PBVI -f ext-oMDP -h $h -m $memory -e 0.01 --p_c 0.1 --p_o 0.001 --p_b 0.01 --lower_bound maxplan --time_max 7200 --num_samples 1000 -w "${world}.dpomdp" --name "ext-pbvi1000_${world}_${h}_${memory}_wcsp"
        echo "Run A*, world=$world, h=$h, please wait ..."
        SDMStudio solve -a A* -f oMDP -h $h -m $memory -e 0.01 --p_c 0.1 --p_o 0.001 --p_b 0.01 --time_max 7200 -w "${world}.dpomdp" --name "A*_${world}_${h}_${memory}_wcsp"
    done
done