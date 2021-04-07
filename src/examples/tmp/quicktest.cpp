#include <iostream>
#include <fstream>
#include <time.h>

#include <sdm/types.hpp>
<<<<<<< HEAD
#include <sdm/parser/parser.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/max_plan_vf.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

// #include <sdm/world/discrete_mdp.hpp>
// #include <sdm/utils/value_function/initializers.hpp>
// #include <sdm/core/state/serialized_state.hpp>
// #include <sdm/world/serialized_mdp.hpp>
// //#include <sdm/world/serialized_occupancy_mdp.hpp>
// #include <sdm/core/state/serialized_occupancy_state.hpp>
// #include <sdm/world/discrete_mdp.hpp>

// #include <sdm/world/occupancy_mdp.hpp>
// #include <sdm/core/state/occupancy_state.hpp>

// #include <sdm/utils/value_function/value_iteration.hpp>
=======
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/algorithms/q_learning.hpp>
>>>>>>> feature/qlearning

using namespace sdm;

int main(int argc, char **argv)
{
    char const *filename;
    if (argc > 1)
    {
        filename = argv[1];
        std::cout << "#> Parsing file \"" << filename << "\"\n";
    }

    else
    {
        std::cerr << "Error: No input file provided." << std::endl;
        return 1;
    }

    try
    {

        number horizon = 2;
        auto beliefMDP = std::make_shared<BeliefMDP<BeliefState, number>>(filename);
        auto lb_init = std::make_shared<MinInitializer<BeliefState, number>>();

        auto lower_bound = std::make_shared<sdm::MappedValueFunction<BeliefState, number>>(beliefMDP, horizon, lb_init);

        lower_bound->initialize();

        auto s = beliefMDP->getInitialState();
        std::cout << *lower_bound << std::endl;

        for (number i = 0; i < 10; i++)
        {
            BeliefState b;
            b[0] = 0.5;
            b[1] = 0.5;
            lower_bound->updateValueAt(b);
            std::cout << *lower_bound << std::endl;
        }

        // for (number a = 0; a < beliefMDP->getUnderlyingProblem()->getActionSpace()->getNumItems(); a++)
        // {
        //     for (number o = 0; o < beliefMDP->getUnderlyingProblem()->getObsSpace()->getNumItems(); o++)
        //     {
        //         std::cout << "A" << a << "O" << o << std::endl;
        //         auto s_ = beliefMDP->nextState(s, a, o);
        //         std::cout << s_ << std::endl;
        //         lower_bound->updateValueAt(s, 1);
        //         std::cout << *lower_bound << std::endl;
        //     }
        // }

        // lower_bound->updateValueAt(s);
        // std::cout << *lower_bound << std::endl;
    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
    return 0;
}
// std::map<int, double> m = {{1, 3.5},
//                            {4, 10}};

// std::shared_ptr<SolvableByHSVI<number, number>> mdp = std::make_shared<DiscreteMDP>(filename);
// std::map<std::string, std::shared_ptr<sdm::Initializer<number, number>> (*)()> m2 = {
//     {"MaxInitializer", &sdm::createInstance<number, number, sdm::MaxInitializer>},
//     {"MinInitializer", &sdm::createInstance<number, number, sdm::MinInitializer>},
// };

// std::apply([](auto&&... args) {((std::cout << args << '\n'), ...);}, t);

//     std::string filePath("../data/world/dpomdp/");

// const int nbfile(1);
// std::string all_file[nbfile] = {"mabc"}; //,"tiger","recycling"};

// std::ofstream myfile;
// myfile.open("resultat.csv");
// myfile << "Filename,Horizon,Case,Discount,Upper_Bound,Lower_Bound,Resultat,Time,Trial \n";

// int max_horizon(2);

// clock_t t_begin, t_end;
// float temps;

// int trials = 10000;

// const int nb_discount(1);
// double all_discount[nb_discount] = {1};

// const int nb_lower_bound(2);
// std::string all_lower_bound[nb_lower_bound] = {"MinInitializer", "BlindInitializer"};

// const int nb_upper_bound(2);
// std::string all_upper_bound[nb_upper_bound] = {"MdpHsviInitializer", "MaxInitializer"};

//number n_agents = 2;

//****************** MMDP
/*
    for(const std::string & filename : all_file)
    {
        
        for(int i(1);i<= max_horizon;++i)
        {
            int horizon(i);
            int length_history(i);

            for(const double & discount : all_discount)
            {
                std::cout<<"*************** MMDP \n";
                std::cout<<"File : "<<filename<<"\n";
                std::cout<<"Horizon : "<<horizon<<"\n";
                std::cout<<"Discount : "<<discount<<"\n";

                myfile<<filename<<","<<horizon<<",MMDP"<<","<<discount;

                //std::cout << "#> Parsing DecPOMDP file \"" << filePath+filename+".dpomdp" << "\"\n";

                using TState = SerializedState<number,number>;
                using TAction = number;

                auto somdp = std::make_shared<SerializedMDP<TState, TAction>>(filePath+filename+".dpomdp");
                
                //number s = 2;
                //SerializedState<number> p(s,std::vector<number>{3});
                //std::cout << p << std::endl;

                //std::cout<<"Min Reward : "<<somdp->getReward()->getMinReward()<<"\n";

                auto hsvi = sdm::algo::makeMappedHSVI<TState, TAction>(somdp, discount, 0, horizon,trials,"tab_mmdp");

                t_begin = clock();


                //std::cout<<"Lower bound : "<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState())<<"\n";
                //std::cout<<"Upper bound : "<<hsvi->getUpperBound()->getValueAt(somdp->getInitialState())<<"\n";
                
                
                hsvi->do_solve();

                t_end = clock();
                // temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;
                // //printf("temps = %f\n", temps);

                ////hsvi->do_test();

                myfile<<","<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState()); 

                // //std::cout<<"Lower bound : "<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState())<<"\n";
                // //std::cout<<"Upper bound : "<<hsvi->getUpperBound()->getValueAt(somdp->getInitialState())<<"\n";

                myfile<<","<<temps<<","<<hsvi->getTrial()<<"\n";
                
            }

        }
    }*/
/*
    for(std::string & filename : all_file)
    {
        for(int i(2);i<= max_horizon;++i)
        {
            int horizon(i);
            int length_history(i);

            for(const double & discount : all_discount)
            {
                std::cout<<"*************** MDP \n";
                std::cout<<"File : "<<filename<<"\n";
                std::cout<<"Horizon : "<<horizon<<"\n";
                std::cout<<"Discount : "<<discount<<"\n";

                myfile<<filename<<","<<horizon<<",MDP"<<","<<discount;


                using TState = number;
                using TAction = number;

                std::string a = filePath+filename+".dpomdp";

                //auto dpomdp_world= sdm::parser::parse_file(filePath+filename+".dpomdp")->toPOMDP()->toMDP();
                auto somdp = std::make_shared<DiscreteMDP>(a);

                auto value = sdm::ValueIteration<TState,TAction>(somdp,discount,0,horizon);

                value.policy_iteration();

                //number s = 2;
                //SerializedState<number> p(s,std::vector<number>{3});
                //std::cout << p << std::endl;

                //std::cout<<"Min Reward : "<<somdp->getReward()->getMinReward()<<"\n";
                auto somdp2 = std::make_shared<DiscreteMDP>(a);
                somdp2->getUnderlyingProblem()->setInternalState(0);

                auto hsvi = sdm::algo::makeMappedHSVI<TState, TAction>(somdp2,"MaxInitializer" ,"MinInitializer",1, 0, horizon,trials,"tab_mdp");

                t_begin = clock();
                
                hsvi->do_initialize();
                hsvi->do_solve();

                t_end = clock();
                // temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;

<<<<<<< HEAD
                //myfile<<","<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState()); 
                myfile<<","<<temps<<","<<hsvi->getTrial()<<"\n";
                
            }

        lower_bound->initialize();
        std::cout << lower_bound->str() << std::endl;

        lower_bound->updateValueAt(beliefMDP->getInitialState(), 0);
        std::cout << lower_bound->size() << std::endl;
        std::cout << lower_bound->str() << std::endl;
    // auto init = sdm::InitializerFactory<number, number>::make("MaxInitializer");
    /*
    std::cout << "Available Init" << std::endl;
    for (auto &v : sdm::InitializerFactory<number, number>::available())
    {
        std::cout << v << std::endl;
    }*/
/*
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
                        //std::cout<<"*************** Extensive \n";
                        std::cout<<"*************** Occupancy \n";

                        std::cout<<"File : "<<filename<<"\n";
                        std::cout<<"Horizon : "<<horizon<<"\n";
                        std::cout<<"Discount : "<<discount<<"\n";

                        std::string name = filename+"#"+std::to_string(horizon)+"#Occupancy#"+std::to_string(discount)+"#"+upper_bound+"#"+lower_bound;

                        myfile<<filename<<","<<horizon<<",Occupancy"<<","<<discount<<","<<upper_bound<<","<<lower_bound;

                        //std::cout << "#> Parsing DecPOMDP file \"" << filePath+filename+".dpomdp" << "\"\n";

                        //using TState = SerializedOccupancyState<SerializedState,JointHistoryTree_p<number>>;
                        //using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;
                        using TState = OccupancyState<number, JointHistoryTree_p<number>>;
                        using TAction = Joint<DeterministicDecisionRule<HistoryTree_p<number>, number>>;

                        //auto somdp = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filePath+filename+".dpomdp", length_history);
                        auto somdp = std::make_shared<OccupancyMDP<TState, TAction>>(filePath+filename+".dpomdp", length_history);

                        //std::cout<<"Min Reward : "<<somdp->getReward()->getMinReward()<<"\n";
                        //std::cout<<"Max Reward : "<<somdp->getReward()->getMaxReward()<<"\n";
=======
    auto environment = std::make_shared<DiscreteMDP>(filename);

    // Instanciate bounds
    std::shared_ptr<sdm::QValueFunction<number, number>> qvalue(new sdm::MappedQValueFunction<number, number>(horizon));
    std::shared_ptr<sdm::QValueFunction<number, number>> target_qvalue(new sdm::MappedQValueFunction<number, number>(horizon));

    QLearning<number, number, number> algo(environment, qvalue, target_qvalue, params...);

    algo->do_solve();
>>>>>>> feature/qlearning

                        auto hsvi = sdm::algo::makeMappedHSVI<TState, TAction>(somdp,upper_bound,lower_bound, discount, 0, horizon,trials,name);
                        hsvi->do_initialize();

                        t_begin = clock();

                        //std::cout<<"Lower bound : "<<hsvi->getLowerBound()->str()<<"\n";
                        //std::cout<<"Upper bound : "<<hsvi->getUpperBound()->str()<<"\n";
                        
                        hsvi->do_solve();

                        t_end = clock();
                        temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;
                        //printf("temps = %f\n", temps);

                        myfile<<","<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState()); 

                        //std::cout<<"Lower bound : "<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState())<<"\n";
                        //std::cout<<"Upper bound : "<<hsvi->getUpperBound()->getValueAt(somdp->getInitialState())<<"\n";

                        myfile<<","<<temps<<","<<hsvi->getTrial()<<"\n";
                    }
                }
            }

        }
    }*/

// for(const std::string & filename : all_file)
// {
//     for(int i(2);i<= max_horizon;++i)
//     {
//         int horizon(i);
//         int length_history(i);
//         for(const double & discount : all_discount)
//         {
//             for(const std::string &upper_bound: all_upper_bound)
//             {
//                 for(const std::string &lower_bound: all_lower_bound)
//                 {
//                     std::cout<<"*************** Extensive \n";
//                     //std::cout<<"*************** Occupancy \n";

//                     std::cout<<"File : "<<filename<<"\n";
//                     std::cout<<"Horizon : "<<horizon<<"\n";
//                     std::cout<<"Discount : "<<discount<<"\n";

//                     std::string name = filename+"#"+std::to_string(horizon)+"#Extensive#"+std::to_string(discount)+"#"+upper_bound+"#"+lower_bound;

//                     myfile<<filename<<","<<horizon<<",Extensive"<<","<<discount<<","<<upper_bound<<","<<lower_bound;

//                     using TState = SerializedOccupancyState<SerializedState,JointHistoryTree_p<number>>;
//                     using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

//                     auto somdp = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filePath+filename+".dpomdp", length_history);

//                     auto hsvi = sdm::algo::makeMappedHSVI<TState, TAction>(somdp,upper_bound,lower_bound, discount, 0, horizon,trials,name);
//                     hsvi->do_initialize();

//                     t_begin = clock();

//                     hsvi->do_solve();

//                     t_end = clock();
//                     temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;

//                     myfile<<","<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState());

//                     myfile<<","<<temps<<","<<hsvi->getTrial()<<"\n";

//                     // ***********************************

//                     std::cout<<"*************** Occupancy \n";
//                     std::cout<<"File : "<<filename<<"\n";
//                     std::cout<<"Horizon : "<<horizon<<"\n";
//                     std::cout<<"Discount : "<<discount<<"\n";

//                     myfile<<filename<<","<<horizon<<",Occupancy"<<","<<discount<<","<<upper_bound<<","<<lower_bound;

//                     using TState2 = OccupancyState<number, JointHistoryTree_p<number>>;
//                     using TAction2 = Joint<DeterministicDecisionRule<HistoryTree_p<number>, number>>;

//                     auto somdp2 = std::make_shared<OccupancyMDP<TState2, TAction2>>(filePath+filename+".dpomdp", length_history);

//                     auto hsvi2 = sdm::algo::makeMappedHSVI<TState2, TAction2>(somdp2,upper_bound,lower_bound, discount, 0, horizon,trials,"tab_hsvi");

//                     hsvi2->do_initialize();

//                     t_begin = clock();

//                     hsvi2->do_solve();

//                     t_end = clock();
//                     temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;
//                     //printf("temps = %f\n", temps);

//                     //hsvi->do_test();

//                     myfile<<","<<hsvi2->getLowerBound()->getValueAt(somdp2->getInitialState());

//                     //std::cout<<"Lower bound : "<<hsvi->getLowerBound()->getValueAt(somdp->getInitialState())<<"\n";
//                     //std::cout<<"Upper bound : "<<hsvi->getUpperBound()->getValueAt(somdp->getInitialState())<<"\n";

//                     myfile<<","<<temps<<","<<hsvi2->getTrial()<<"\n";
//                 }
//             }
//         }

// }