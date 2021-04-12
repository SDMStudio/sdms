
#pragma once

#include <iostream>
#include <fstream>
#include <time.h>

#include <sdm/algorithms.hpp>


namespace sdm
{
    void test(std::string filepath,std::vector<std::string> all_formalism, std::vector<std::string> all_problem,std::vector<int> all_horizon,std::vector<double> all_discount,
    std::vector<std::string> all_lower_bound,std::vector<std::string> all_upper_bound) 
    {
        std::ofstream myfile;
        clock_t t_begin, t_end;
        double temps;

        for(const std::string & filename : all_problem)
        {
            std::string filepath_ = filepath+filename+".dpomdp";
            for(const int &i :all_horizon)
            {
                int horizon(i);
                int length_history(i);
                for(const double & discount : all_discount)
                {
                    for(const std::string &upper_bound: all_upper_bound)
                    {
                        for(const std::string &lower_bound: all_lower_bound)
                        {
                            for(const std::string &formalism : all_formalism)
                            {
                                std::string name = filename+"#"+std::to_string(horizon)+"#"+formalism+"#"+std::to_string(discount)+"#"+upper_bound+"#"+lower_bound;
                                myfile<<filename<<","<<horizon<<","<<formalism<<","<<discount<<","<<upper_bound<<","<<lower_bound;
                                
                                std::cout<<"\n*************** \n";
                                std::cout<<"Formalism : "<<formalism<<"\n";
                                std::cout<<"File : "<<filename<<"\n";
                                std::cout<<"Horizon : "<<horizon<<"\n";
                                std::cout<<"Discount : "<<discount<<"\n";   
                                std::cout<<"Upper_bound : "<<upper_bound<<"\n";
                                std::cout<<"Lower_bound : "<<lower_bound<<"\n";
                                std::cout<<"\n*************** \n";
                                
                                try
                                {
                                    auto algo = sdm::algo::make("hsvi",filepath_,formalism,"","",upper_bound,lower_bound,discount,0,horizon,10000,name);

                                    //auto value = sdm::ValueIteration<TState,TAction>(smdp,discount,0,horizon);
                                    t_begin = clock();

                                    algo->do_initialize();
                                    algo->do_solve();

                                    t_end = clock();
                                    temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;

                                    //myfile<<","<<auto->getResultOpti();

                                    //myfile<<","<<temps<<","<<auto->getTrial()<<"\n";
                                }
                                catch (sdm::exception::Exception &e)
                                {
                                    std::cout << "!!! Exception: " << e.what() << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}