
#pragma once

#include <iostream>
#include <fstream>
#include <time.h>

#include <sdm/algorithms.hpp>
#include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{
    /**
     * @brief This function is utilized to do precise large scale experimentation
     * 
     * @param filepath 
     * @param all_formalism a vector of all formalism to use (mdp/pomdp/...)
     * @param all_problem a vector of all problem to be solved ( tiger/mabc/...)
     * @param all_horizon a vector of all planning horizon 
     * @param all_discount a vector of all discount factor
     * @param all_lower_bound a vector of all lower_bound
     * @param all_upper_bound a vector of all upper bound
     * @param mean the number of repetition
     * @param sauvegarde_path the path to save all the data
     */
    void test(std::vector<std::string> all_formalism={"mdp"}, std::vector<std::string> all_problem={"mabc"},std::vector<int> all_horizon={2},
        std::vector<double> all_discount={1},std::vector<std::string> upper_bound_name = {""},std::vector<std::string> lower_bound_name={""},
        std::vector<std::string> all_lower__init_name={"MinInitializer"},std::vector<std::string> all_upper_init_name= {"MaxInitializer"}, std::vector<int> all_truncation = {2},
        std::vector<std::string> all_sawtooth_current_type_of_resolution = {"BigM"}, std::vector<number> all_sawtooth_BigM = {1000},std::vector<std::string> all_sawtooth_type_of_linear_program = {"Full"},
        std::vector<TypeOfMaxPlanPrunning> all_type_of_maxplan_prunning = {TypeOfMaxPlanPrunning::PAIRWISE} ,std::vector<int> all_freq_prunning_lower_bound = {-1},
        std::vector<TypeOfSawtoothPrunning> all_type_of_sawtooth_pruning = {TypeOfSawtoothPrunning::NONE}, std::vector<int> all_freq_prunning_upper_bound = {-1},
        int mean = 2, std::string filepath = "../data/world/dpomdp/",std::string save_path = "../../run/Resultat/resultat.csv") 
    {
        auto csv_logger = std::make_shared<sdm::CSVLogger>(save_path, std::vector<std::string>{"Filename","Horizon","Discount","Upper_init","Lower_init","Upper_bound","Lower_bound","truncation","sawtooth_type_of_resolution", "sawtooth_BigM","sawtooth_type_of_linear_program","type_of_maxplan_prunning","freq_prunning_lower_bound","type_of_sawtooth_prunning","freq_prunning_upper_bound","Problem","Resultat","Time","Trial"});
        auto logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{csv_logger});

        std::ofstream myfile;

        std::chrono::high_resolution_clock::time_point t_begin, t_end;

        for(const std::string & filename : all_problem)
        {
            std::string filepath_ = filepath+filename+".ndpomdp";
            for(const int &horizon :all_horizon)
            {
                for(const double & discount : all_discount)
                {
                    for(const std::string &upper_bound_init: all_upper_init_name)
                    {
                        for(const std::string &lower_bound_init: all_lower__init_name)
                        {
                            for(const std::string &upper_bound: upper_bound_name)
                            {
                                for(const std::string &lower_bound: lower_bound_name)
                                {
                                    for(const int &truncation: all_truncation)
                                    {
                                        for(const std::string &sawtooth_current_type_of_resolution: all_sawtooth_current_type_of_resolution)
                                        {
                                            for(const number &sawtooth_BigM: all_sawtooth_BigM)
                                            {
                                                for(const std::string sawtooth_type_of_linear_program: all_sawtooth_type_of_linear_program)
                                                {
                                                    for(const int freq_prunning_lower_bound : all_freq_prunning_lower_bound)
                                                    {
                                                        for(const TypeOfMaxPlanPrunning type_of_maxplan_prunning : all_type_of_maxplan_prunning)
                                                        {
                                                            for(const int freq_prunning_upper_bound : all_freq_prunning_upper_bound)
                                                            {
                                                                for(const TypeOfSawtoothPrunning type_of_sawtooth_pruning : all_type_of_sawtooth_pruning)
                                                                {
                                                                    for(const std::string &formalism : all_formalism)
                                                                    {                                                        
                                                                        std::vector<float> times;
                                                                        std::vector<float> trials;
                                                                        std::vector<double> results;
                                                                        for(int i =0; i<mean;i++)
                                                                        {
                                                                            std::string name = filename+"#"+std::to_string(horizon)+"#"+formalism+"#"+std::to_string(discount)+"#"+upper_bound_init+"#"+lower_bound_init+"#"+upper_bound+"#"+lower_bound+"#"+std::to_string(truncation)+"#"+sawtooth_current_type_of_resolution+"#"+std::to_string(sawtooth_BigM)+"#"+sawtooth_type_of_linear_program+"#"+std::to_string(type_of_maxplan_prunning)+"#"+std::to_string(freq_prunning_lower_bound)+"#"+std::to_string(type_of_sawtooth_pruning)+"#"+std::to_string(freq_prunning_upper_bound)+"#"+std::to_string(i);

                                                                            std::cout<<"\n*************** \n";
                                                                            std::cout<<"Formalism : "<<formalism<<"\n";
                                                                            std::cout<<"File : "<<filename<<"\n";
                                                                            std::cout<<"Horizon : "<<horizon<<"\n";
                                                                            std::cout<<"Discount : "<<discount<<"\n";   
                                                                            std::cout<<"Upper_bound_init : "<<upper_bound_init<<"\n";
                                                                            std::cout<<"Lower_bound_init : "<<lower_bound_init<<"\n";
                                                                            std::cout<<"Upper_bound : "<<upper_bound<<"\n";
                                                                            std::cout<<"Lower_bound : "<<lower_bound<<"\n";
                                                                            std::cout<<"truncation : "<<truncation<<"\n";
                                                                            std::cout<<"sawtooth_current_type_of_resolution : "<<sawtooth_current_type_of_resolution<<"\n";
                                                                            std::cout<<"sawtooth_BigM : "<<sawtooth_BigM<<"\n";
                                                                            std::cout<<"sawtooth_type_of_linear_program : "<<sawtooth_type_of_linear_program<<"\n";
                                                                            std::cout<<"freq_prunning_lower_bound : "<<freq_prunning_lower_bound<<"\n";
                                                                            std::cout<<"type_of_maxplan_prunning : "<<type_of_maxplan_prunning<<"\n";
                                                                            std::cout<<"freq_prunning_upper_bound : "<<freq_prunning_upper_bound<<"\n";
                                                                            std::cout<<"type_of_sawtooth_pruning : "<<type_of_sawtooth_pruning<<"\n";
                                                                            std::cout<<"Try : "<<i<<"\n";
                                                                            std::cout<<"\n*************** \n";
                                                                            
                                                                            try
                                                                            {
                                                                                auto algo = sdm::algo::make("hsvi",filepath_,formalism,upper_bound,lower_bound,upper_bound_init,lower_bound_init,discount,0.01,horizon,5000,truncation,name,1000,sawtooth_current_type_of_resolution,sawtooth_BigM,sawtooth_type_of_linear_program,type_of_maxplan_prunning,freq_prunning_lower_bound,type_of_sawtooth_pruning,freq_prunning_upper_bound);

                                                                                //auto value = sdm::ValueIteration<TState,TAction>(smdp,discount,0,horizon);
                                                                                algo->initialize();
                                                                                t_begin = std::chrono::high_resolution_clock::now();

                                                                                algo->solve();

                                                                                t_end = std::chrono::high_resolution_clock::now();

                                                                                times.push_back(std::chrono::duration_cast<std::chrono::duration<double>>(t_end-t_begin).count());
                                                                                trials.push_back(algo->getTrial());
                                                                                results.push_back(algo->getResult());

                                                                            }
                                                                            catch (sdm::exception::Exception &e)
                                                                            {
                                                                                std::cout << "!!! Exception: " << e.what() << std::endl;
                                                                                // problem.push_back(name);
                                                                            }
                                                                        }
                                                                        logger_->log(filename,horizon,discount,upper_bound_init,lower_bound_init,upper_bound,lower_bound,truncation,sawtooth_current_type_of_resolution,sawtooth_BigM,sawtooth_type_of_linear_program,type_of_maxplan_prunning,freq_prunning_lower_bound,type_of_sawtooth_pruning,freq_prunning_upper_bound,formalism,std::accumulate( results.begin(), results.end(), 0.0) / results.size(),std::accumulate( times.begin(), times.end(), 0.0) / times.size(),std::accumulate( trials.begin(), trials.end(), 0.0) / trials.size());
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}