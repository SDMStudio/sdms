#include <iostream>
#include <fstream>
#include <time.h>

#include <sdm/types.hpp>
#include <sdm/tools.hpp>
#include <sdm/utils/struct/pair.hpp>
// #include <sdm/utils/logging/logger.hpp>
// #include <sdm/utils/struct/vector.hpp>
// #include <sdm/utils/struct/pair.hpp>
// #include <sdm/utils/struct/tuple.hpp>
// #include <sdm/world/world_type.hpp>
// #include <sdm/world/discrete_mdp.hpp>
// #include <sdm/world/discrete_pomdp.hpp>
// #include <sdm/world/discrete_decpomdp.hpp>
// #include <sdm/world/solvable_by_hsvi.hpp>
// #include <sdm/world/belief_mdp.hpp>
// #include <sdm/world/serialized_occupancy_mdp.hpp>

// #include <sdm/utils/linear_algebra/mapped_vector.hpp>
// #include <sdm/world/ndpomdp.hpp>
// //#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/core/state/serialized_state.hpp>
#include <sdm/world/serialized_mdp.hpp>


// #include <sdm/types.hpp>
// #include <sdm/tools.hpp>
// //#include <sdm/utils/logging/logger.hpp>
// //#include <sdm/utils/struct/vector.hpp>
// #include <sdm/utils/struct/pair.hpp>
// #include <sdm/utils/struct/tuple.hpp>
 #include <sdm/algorithms.hpp>
// #include <sdm/parser/parser.hpp>

// #include <sdm/utils/linear_algebra/mapped_vector.hpp>
// #include <sdm/world/ndpomdp.hpp>
// #include <sdm/world/serialized_occupancy_mdp.hpp>
//#include <fmt/format.h>

using namespace sdm;

int main(int argc, char **argv)
{
    /*
    std::string filename;
    number horizon, length_history;

    if (argc > 2)
    {
        filename = argv[1];
        horizon = std::atoi(argv[2]);
        length_history = horizon;

        if (argc > 3)
        {
            length_history = std::atoi(argv[3]);
        }
    }
    else
    {
        std::cerr << "Error:  arg[1] must be an input file, arg[2] must be the horizon, arg[3] is optional (the length of history)." << std::endl;
        return 1;
    }
    // using TState = SerializedOccupancyState<number, JointHistoryTree_p<number>>;
    // using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;
    // std::shared_ptr<SolvableByHSVI<TState, TAction>> mdp = std::make_shared<SerializedOccupancyMDP<>>(filename);

    // auto upb = mdp->getUnderlyingProblem();

    // std::cout << *upb->getStateSpace() << std::endl;
    // std::cout << *upb->getActionSpace() << std::endl;
    // std::cout << *upb << std::endl;

    sdm::Pair<number, number> p = std::make_pair(2,3);

    std::cout << p ;
    // print(3, 4.5, "hello");

    // std::apply([](auto&&... args) {((std::cout << args << '\n'), ...);}, t);

    std::string filePath("../data/world/dpomdp/");

    const int nbfile(1);
    std::string all_file[nbfile] = {"mabc"};//,"tiger","recycling"};


    std::ofstream myfile;
    myfile.open("resultat.csv");
    myfile<<"Filename,Horizon,Case,Discount,Resultat,Time,Trial \n";

    int max_horizon(3);

    clock_t t_begin, t_end;
    float temps;

    int trials = 10000; 

    const int nb_discount(1);
    double all_discount[nb_discount] = {1};

    //number n_agents = 2;

    //****************** MMDP
    
    for(const std::string & filename : all_file)
    {

        for(int i(1);i<= max_horizon;++i)
        {
            int horizon(i);
            int length_history(i);

            for(const double & discount : all_discount)
            {
                std::cout<<"*************** Extensive \n";
                std::cout<<"File : "<<filename<<"\n";
                std::cout<<"Horizon : "<<horizon<<"\n";
                std::cout<<"Discount : "<<discount<<"\n";

                myfile<<filename<<","<<horizon<<",Extensive"<<","<<discount;

                //std::cout << "#> Parsing DecPOMDP file \"" << filePath+filename+".dpomdp" << "\"\n";

                using TState = SerializedState<number>;
                using TAction = number;

                auto somdp = std::make_shared<SerializedMDP<TState, TAction>>(filePath+filename+".dpomdp");
                
                //number s = 2;
                //SerializedState<number> p(s,std::vector<number>{3});
                //std::cout << p << std::endl;

                //std::cout<<"Min Reward : "<<somdp->getReward()->getMinReward()<<"\n";
                //std::cout<<"Max Reward : "<<somdp->getReward()->getMaxReward()<<"\n";

                 auto hsvi = sdm::algo::makeMappedHSVI<TState, TAction>(somdp, discount, 0, horizon,trials,"tab_hsvi",somdp->getNumberAgent());

                // t_begin = clock();


                std::cout<<"Lower bound : "<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState())<<"\n";
                std::cout<<"Upper bound : "<<hsvi->getUpperBound()->getValueAt(somdp->getInitialState())<<"\n";
                
                
                hsvi->do_solve();

                // t_end = clock();
                // temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;
                // //printf("temps = %f\n", temps);

                // //hsvi->do_test();

                myfile<<","<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState()); 

                // //std::cout<<"Lower bound : "<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState())<<"\n";
                // //std::cout<<"Upper bound : "<<hsvi->getUpperBound()->getValueAt(somdp->getInitialState())<<"\n";

                myfile<<","<<temps<<","<<hsvi->getTrial()<<"\n";
            }

        }

    }
    


    //****************** Extensive
    /*
    for(const std::string & filename : all_file)
    {

        for(int i(1);i<= max_horizon;++i)
        {
            int horizon(i);
            int length_history(i);

            for(const double & discount : all_discount)
            {
                std::cout<<"*************** Extensive \n";
                std::cout<<"File : "<<filename<<"\n";
                std::cout<<"Horizon : "<<horizon<<"\n";
                std::cout<<"Discount : "<<discount<<"\n";

                myfile<<filename<<","<<horizon<<",Extensive"<<","<<discount;

                //std::cout << "#> Parsing DecPOMDP file \"" << filePath+filename+".dpomdp" << "\"\n";

                using TState = SerializedOccupancyState<number,JointHistoryTree_p<number>>;
                using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

                auto somdp = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filePath+filename+".dpomdp", length_history);

                //std::cout<<"Min Reward : "<<somdp->getReward()->getMinReward()<<"\n";
                //std::cout<<"Max Reward : "<<somdp->getReward()->getMaxReward()<<"\n";

                auto hsvi = sdm::algo::makeMappedHSVI<TState, TAction>(somdp, discount, 0, horizon,trials,"tab_hsvi",somdp->getNumberAgent());

                t_begin = clock();

                hsvi->do_solve();

                t_end = clock();
                temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;
                //printf("temps = %f\n", temps);

                //hsvi->do_test();

                myfile<<","<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState()); 

                //std::cout<<"Lower bound : "<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState())<<"\n";
                //std::cout<<"Upper bound : "<<hsvi->getUpperBound()->getValueAt(somdp->getInitialState())<<"\n";

                myfile<<","<<temps<<","<<hsvi->getTrial()<<"\n";
            }

        }

    }*/

    // ************************** Simul           

    /*
    for(const std::string & filename : all_file)
    {
        for(int i(1);i<= max_horizon;++i)
        {

            int horizon(i);
            int length_history(i);

            for(const double & discount : all_discount)
            {
                std::cout<<"*************** Simultane \n";
                std::cout<<"File : "<<filename<<"\n";
                std::cout<<"Horizon : "<<horizon<<"\n";
                std::cout<<"Discount : "<<discount<<"\n";

                myfile<<filename<<","<<horizon<<",Simultane"<<","<<discount;


                using TActionPrescriptor = Joint<DeterministicDecisionRule<HistoryTree_p<number>, number>>;
                using TStatePrescriptor = OccupancyState<number, JointHistoryTree_p<number>>;

                auto oMDP = std::make_shared<OccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(filePath+filename+".dpomdp", horizon);
                auto hsvi2 = sdm::algo::makeMappedHSVI<TStatePrescriptor, TActionPrescriptor>(oMDP, discount, 0.0, horizon,trials);
                

                t_begin = clock();

                //std::cout<<"Min Reward : "<<oMDP->getReward()->getMinReward()<<"\n";
                //std::cout<<"Max Reward : "<<oMDP->getReward()->getMaxReward()<<"\n";

                //std::cout<<"Lower bound : "<<hsvi2->getLowerBound()->getValueAt(oMDP->getInitialState())<<"\n";
                //std::cout<<"Upper bound : "<<hsvi2->getUpperBound()->getValueAt(oMDP->getInitialState())<<"\n";

                hsvi2->do_solve();

                t_end = clock();
                temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;
                //printf("temps = %f\n", temps);

                //hsvi->do_test();

                myfile<<","<<hsvi2->getLowerBound()->getValueAt(oMDP->getInitialState()); 
                myfile<<","<<temps<<","<<hsvi2->getTrial()<<" \n";
            }
        }
    }*/
    
    // clock_t t_begin, t_end;

    // std::cout << "#> Parsing DecPOMDP file \"" << filename << "\"\n";
    // number n_agents = 2;

    // using TState = SerializedOccupancyState<number, JointHistoryTree_p<number>>;
    // using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

    // auto somdp = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filename, length_history);

    // auto hsvi = sdm::algo::makeMappedHSVI<TState, TAction>(somdp, 1.0, 0.1, horizon * n_agents);

    // t_begin = clock();

    // hsvi->do_solve();

    // t_end = clock();
    // float temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;
    // printf("temps = %f\n", temps);

    // hsvi->do_test();

    // NDPOMDP ndpomdp(filename);

    // std::cout << "--------------------------------" << std::endl;

    // std::cout << ndpomdp.getStateSpace() << std::endl;
    // std::cout << ndpomdp.getActionSpace() << std::endl;
    // std::cout << ndpomdp.getObsSpace() << std::endl;

    return 0;
}