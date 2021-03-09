#include <iostream>
#include <cassert>

#include <tuple>
#include <typeinfo>
#include <sdm/common.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/algorithms.hpp>

#include <sdm/world/decision_process.hpp>
#include <sdm/world/po_decision_process.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_mmdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>

#include <sdm/core/state/state.hpp>
#include <sdm/core/reward.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/observation_dynamics.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename;

    if (argc > 1)
    {
        filename = argv[1];
        std::cout << "#> Parsing NDPOMDP file \"" << filename << "\"\n";
    }

    else
    {
        std::cerr << "Error: No input file provided." << std::endl;
        return 1;
    }

    double discount = 0.99, error = 0.001;
    int horizon = 5, trials = 1000;

    auto problem = std::make_shared<DiscreteMDP>(filename);

    // std::cout << problem->toStdFormat() << std::endl;
    // std::cout << problem->toXML() << std::endl;

    // auto problem = sdm::parser::parse_file(filename);
    std::cout << "State space : " << *problem->getStateSpace() << std::endl;
    std::cout << "Action space : " << *problem->getActionSpace() << std::endl;
    std::cout << "Observation space : " << *problem->getObsSpace() << std::endl;
    std::cout << "Internal State : " << problem->getInternalState() << std::endl;
    std::cout << "Planning Horizon : " << problem->getPlanningHorizon() << std::endl;
    std::cout << "Discount factor : " << problem->getDiscount() << std::endl;
    std::cout << "Criterion : " << problem->getCriterion() << std::endl;
    std::cout << "Filename : " << problem->getFileName() << std::endl;
    std::cout << "Bound : " << problem->getBound() << std::endl;
    std::cout << "Transition : " << problem->getStateDynamics()->getTransitionProbability(0, 0, 0) << std::endl;

    std::cout << "Start Distrib : ";
    for (auto proba : problem->getStartDistrib().probabilities())
        std::cout << proba << "\t";
    std::cout << "\n\n";

    // auto mdp = problem->toPOMDP()->toMDP();
    // auto occupancy_mdp = problem->toPOMDP()->toBeliefMDP();
    // auto belief_mdp = problem->toOccupancyMDP();

    auto hsvi = sdm::algo::makeMappedHSVI<number, number>(problem, discount, error, horizon, trials);

    // for (auto &state : problem->getStateSpace()->getAll())
    // {
    //     problem->setInternalState(state);
    //     hsvi->do_solve();
    // }
    
    hsvi->do_solve();
    hsvi->do_test();

    // std::unordered_map<int, int> test;
    // test.emplace(3, 4);
    // test.emplace(4, 4);
    // test.emplace(5, 3);

    // for (const auto &value : test)
    // {
    //     std::cout << value.first << " - " << value.second << std::endl;
    // }

    // std::unordered_map<Joint<number>, int> test2;
    // test2.insert({Joint<number>({1, 2, 3}), 4});
    // test2.insert({Joint<number>({1, 3, 3}), 4});
    // test2.insert({Joint<number>({1, 5, 3}), 3});

    // for (const auto &value : test2)
    // {
    //     std::cout << value.first << " - " << value.second << std::endl;
    // }

    // decision.setupDynamicsGenerator();
    // std::cout << decision.reset() << std::endl;
    // auto feedback = decision.step(0);
    // std::cout << "("<< std::get<0>(feedback) << ", " << std::get<1>(feedback) << ", " << std::get<2>(feedback) << ")"<< std::endl;

    // feedback = decision.step(2);
    // std::cout << "("<< std::get<0>(feedback) << ", " << std::get<1>(feedback) << ", " << std::get<2>(feedback) << ")"<< std::endl;

    // feedback = decision.step(1);
    // std::cout << "("<< std::get<0>(feedback) << ", " << std::get<1>(feedback) << ", " << std::get<2>(feedback) << ")"<< std::endl;

    // MultiDiscreteSpace<number> ds1({3, 4});
    // MultiDiscreteSpace<std::string> ds2({{"Hi", "Hello"}, {"Bye", "ciao"}});

    // std::cout << ds1 << "\n" << ds2 << std::endl;
    // std::cout << decision.step(1) << std::endl;
    // std::cout << decision.step(2) << std::endl;
    // std::cout << decision.step(0) << std::endl;

    // std::cout << process.getStateSpace() << std::endl;

    // process.setStartDistrib({0.4, 0.1, 0.5});
    // for (double x :  process.getStartDistrib().probabilities())
    //     std::cout << x << std::endl;

    // std::cout << process.init() << std::endl;
    // std::cout << process.getStartDistrib()(sdm::common::global_urng()) << std::endl;
    // // std::cout << process.nextState() << std::endl;

    // PartiallyObservableProcess po_process(DiscreteSpace(3), DiscreteSpace(4), std::discrete_distribution<number>({0.2, 0.1, 0.3}));

    // std::cout << po_process.init() << std::endl;
    // std::cout << po_process.getStartDistrib()(sdm::common::global_urng()) << std::endl;

    // std::cout << po_process.getObsSpace() << std::endl;
    // std::cout << po_process.getStateSpace() << std::endl;

    // StochasticProcess<ContinuousSpace, std::normal_distribution<tinteger>> process2;

    // std::cout << process2.init() << std::endl;
    // std::cout << process2.nextState() << std::endl;

    // template <typename DState, typename DAction>
    // using DecisionProcess = DecisionProcess<>;

    return 0;
}