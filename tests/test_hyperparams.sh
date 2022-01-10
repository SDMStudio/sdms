#!/bin/bash

memory=1
max_horizon=4
worlds=('mabc' 'recycling' 'tiger' 'alignment_2x4' 'boxPushingUAI07' 'GridSmall' 'Mars')
memory=('1' '1' '3' '3' '2' '2' '2')
 
 echo ${#worlds[@]}
for (( h=2; h<=$max_horizon; h++ ))
do 
    for (( i=0; i<${#worlds[@]}; i++ ))
    do
        # echo "Run HSVI, ${worlds[$i]}, h=$h, mem=${memory[$i]}, please wait ..."
        # SDMStudio solve -a HSVI -f oMDP -h $h -m ${memory[$i]} -e 0.01 --p_c 0.1 --p_o 0.01 --p_b 0.001 --lower_bound maxplan_wcsp --lb_freq_pruning 10 --lb_type_of_pruning pairwise --upper_bound sawtooth_lp --ub_init Pomdp --ub_freq_pruning 10 --ub_type_of_pruning pairwise --time_max 7200 -w "${worlds[$i]}.dpomdp" --name "hsvi_${worlds[$i]}_${h}_wcsp_lp"
        # echo "Run sequential HSVI, world=$world, h=$h, please wait ..."
        # SDMStudio solve -a HSVI -f ext-oMDP -h $h -m ${memory[$i]} -e 0.01 --p_c 0.1 --p_o 0.01 --p_b 0.001 --lower_bound maxplan --lb_freq_pruning 10 --lb_type_of_pruning bounded --upper_bound sawtooth_lp --ub_init Pomdp --time_max 7200 -w "${world}.dpomdp" --name "ext-hsvi_${world}_${h}_wcsp_lp"
        echo "Run QLearning, world=${worlds[$i]}, h=$h, please wait ..."
        SDMStudio solve -a QLearning -f oMDP -h $h -m ${memory[$i]} -e 0.1 --p_c 0.1 --p_o 0.01 --p_b 0.001 --lower_bound maxplan_wcsp --lb_init Min --time_max 14400 -t 1000000 -w "${worlds[$i]}.dpomdp" --name "qlearning_${worlds[$i]}_${h}_wcsp" --store_actions false
        # SDMStudio solve -a PBVI -f oMDP -h $h -m ${memory[$i]} -e 0.01 --p_c 0.1 --p_o 0.01 --p_b 0.001 --lower_bound maxplan_wcsp --time_max 7200 --num_samples 100 -w "${worlds[$i]}.dpomdp" --name "pbvi10000_${worlds[$i]}_${h}_wcsp"
        # echo "Run sequential PBVI, world=$world, h=$h, please wait ..."
        # SDMStudio solve -a PBVI -f ext-oMDP -h $h -m ${memory[$i]} -e 0.01 --p_c 0.1 --p_o 0.01 --p_b 0.001 --lower_bound maxplan --time_max 7200 --num_samples 100 -w "${world}.dpomdp" --name "ext-pbvi100_${world}_${h}_wcsp"
        # echo "Run PBVI, world=$world, h=$h, please wait ..."
        # SDMStudio solve -a PBVI -f oMDP -h $h -m ${memory[$i]} -e 0.01 --p_c 0.1 --p_o 0.01 --p_b 0.001 --lower_bound maxplan_wcsp --time_max 7200 --num_samples 1000 -w "${world}.dpomdp" --name "pbvi1000_${world}_${h}_wcsp"
        # echo "Run sequential PBVI, world=$world, h=$h, please wait ..."
        # SDMStudio solve -a PBVI -f ext-oMDP -h $h -m ${memory[$i]} -e 0.01 --p_c 0.1 --p_o 0.01 --p_b 0.001 --lower_bound maxplan --time_max 7200 --num_samples 1000 -w "${world}.dpomdp" --name "ext-pbvi1000_${world}_${h}_wcsp"
        # echo "Run A*, world=$world, h=$h, please wait ..."
        # SDMStudio solve -a A* -f oMDP -h $h -m ${memory[$i]} -e 0.01 --p_c 0.1 --p_o 0.01 --p_b 0.001 --time_max 7200 -w "${world}.dpomdp" --name "A*_${world}_${h}_wcsp"
    done
done