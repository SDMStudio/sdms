#include <iostream>
#include <fstream>
#include <time.h>

#include <sdm/types.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/max_plan_vf.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

#include <sdm/algorithms.hpp>

#include <sdm/utils/value_function/initializers.hpp>

#include <sdm/core/state/serialized_belief_state.hpp>
#include <sdm/world/serialized_belief_mdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // try
    // {
    //     using TState = OccupancyState<number, JointHistoryTree_p<number>>;
    //     using TAction = Joint<DeterministicDecisionRule<HistoryTree_p<number>, number>>;

    //     auto somdp = std::make_shared<OccupancyMDP<TState, TAction>>(filePath+filename+".dpomdp", length_history);

    //     number horizon = 2;
    //     auto beliefMDP = std::make_shared<BeliefMDP<TState, TAction>>(filename);
    //     auto lb_init = std::make_shared<MinInitializer<TState, TAction>>();

    //     auto lower_bound = std::make_shared<sdm::MaxPlanValueFunction<TState, TAction>>(beliefMDP, horizon, lb_init);
    // }
    // catch (sdm::exception::Exception &e)
    // {
    //     std::cout << "!!! Exception: " << e.what() << std::endl;
    // }
    std::string filePath("../data/world/dpomdp/");

    const int nbfile(1);
    std::string all_file[nbfile] = {"mabc"}; //,"tiger","recycling"};

    std::ofstream myfile;

    int max_horizon(2);

    clock_t t_begin, t_end;
    float temps;

    int trials = 10000;

    const int nb_discount(1);
    double all_discount[nb_discount] = {1};

    const int nb_lower_bound(1);
    std::string all_lower_bound[nb_lower_bound] = {"MinInitializer"};

    const int nb_upper_bound(1);
    std::string all_upper_bound[nb_upper_bound] = {"MaxInitializer"};

//****************** MMDP
// using TState = SerializedState<number,number>;
// using TAction = number;

// auto somdp = std::make_shared<SerializedMDP<TState, TAction>>(filePath+filename+".dpomdp");

//******************* MDP 

// using TState = number; //<number, number>;
// using TAction = number;

// auto mmdp = std::make_shared<DiscreteMDP>(filePath+filename+".dpomdp");

    
    for(const std::string & filename : all_file)
    {
        for(int i(2);i<= max_horizon;++i)
        {
            int horizon(i);
            int length_history(i);
            for(const double & discount : all_discount)
            {
                for(const std::string &upper_bound: all_upper_bound)
                {
                    for(const std::string &lower_bound: all_lower_bound)
                    {
                        std::string name = filename+"#"+std::to_string(horizon)+"#Extensive#"+std::to_string(discount)+"#"+upper_bound+"#"+lower_bound;

                        try
                        {
                            using TState = SerializedBeliefState;
                            using TAction = number;
                            using TObservation = number;

                            auto somdp = std::make_shared<SerializedBeliefMDP<TState, TAction,TObservation>>(filePath+filename+".dpomdp", length_history);

                            auto hsvi = sdm::algo::makeHSVI<TState, TAction>(somdp,"tabular","maxplan",upper_bound,lower_bound, discount, 0, horizon,trials,name);

                            hsvi->do_initialize();

                            hsvi->do_solve();

                        }
                        catch (sdm::exception::Exception &e)
                        {
                            std::cout << "!!! Exception: " << e.what() << std::endl;
                        }

                        /*
                        std::cout<<"*************** Extensive \n";

                        std::cout<<"File : "<<filename<<"\n";
                        std::cout<<"Horizon : "<<horizon<<"\n";
                        std::cout<<"Discount : "<<discount<<"\n";

                        myfile<<filename<<","<<horizon<<",Extensive"<<","<<discount<<","<<upper_bound<<","<<lower_bound;

                        using TState = SerializedOccupancyState<SerializedState,JointHistoryTree_p<number>>;
                        using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

                        auto somdp = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filePath+filename+".dpomdp", length_history);

                        auto hsvi = sdm::algo::makeMappedHSVI<TState, TAction>(somdp,upper_bound,lower_bound, discount, 0, horizon,trials,name);
                        hsvi->do_initialize();

                        t_begin = clock();

                        hsvi->do_solve();

                        t_end = clock();
                        temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;

                        myfile<<","<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState());

                        myfile<<","<<temps<<","<<hsvi->getTrial()<<"\n";

                        // ***********************************

                        name = filename+"#"+std::to_string(horizon)+"#Simultane#"+std::to_string(discount)+"#"+upper_bound+"#"+lower_bound;

                        myfile<<filename<<","<<horizon<<",Occupancy"<<","<<discount<<","<<upper_bound<<","<<lower_bound;

                        myfile<<filename<<","<<horizon<<",Occupancy"<<","<<discount<<","<<upper_bound<<","<<lower_bound;

                        using TState2 = OccupancyState<number, JointHistoryTree_p<number>>;
                        using TAction2 = Joint<DeterministicDecisionRule<HistoryTree_p<number>, number>>;

                        auto hsvi2 = sdm::algo::makeMappedHSVI<TState2, TAction2>(somdp2,upper_bound,lower_bound, discount, 0, horizon,trials,name);

                        auto hsvi2 = sdm::algo::makeMappedHSVI<TState2, TAction2>(somdp2,upper_bound,lower_bound, discount, 0, horizon,trials,"tab_hsvi");

                        hsvi2->do_initialize();

                        t_begin = clock();

                        hsvi2->do_solve();

                        t_end = clock();
                        temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;
                        //printf("temps = %f\n", temps);

                        //hsvi->do_test();

                        myfile<<","<<hsvi2->getLowerBound()->getValueAt(somdp2->getInitialState());
                        */
                    }
                }
            }
        }
    }
}