#include <iostream>
#include <cassert>

#include <tuple>
#include <typeinfo>
#include <sdm/common.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/world/decision_process.hpp>
#include <sdm/world/po_decision_process.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_mmdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>

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

    auto problem = sdm::parser::parse_file(filename);

    auto state_sp = std::make_shared<DiscreteSpace<number>>(2);
    auto action_sp = std::make_shared<MultiDiscreteSpace<number>>(std::vector<number>{2, 2, 2});
    // auto action_sp = std::make_shared<DiscreteSpace<number>>(3);
    auto obs_sp = std::make_shared<MultiDiscreteSpace<number>>(std::vector<number>{8, 2, 3});
    auto state_dyn = std::make_shared<StateDynamics>(action_sp->getNumItems(), state_sp->getNumItems());
    auto obs_dyn = std::make_shared<ObservationDynamics>(action_sp->getNumItems(), obs_sp->getNumItems(), state_sp->getNumItems());

    // std::shared_ptr<std::vector<Reward>> rews_fct = std::make_shared<std::vector<Reward>>();
    // for (int i = 0; i < action_sp->getNumSpaces(); i++)
    // {
    //     rews_fct->push_back(Reward(state_sp->getNumItems(), action_sp->getNumItems()));
    // }
    auto reward_f = std::make_shared<Reward>(state_sp->getNumItems(), action_sp->getNumItems());

    DiscreteDecPOMDP decision(state_sp, action_sp, obs_sp, state_dyn, obs_dyn, reward_f, {0.3, 0.7}, 2);
    // decision.setDiscount(0.8);
    // decision.setCriterion((Criterion)0);
    // DiscreteMDP decision(state_sp, action_sp, state_dyn, reward_f, {0.3, 0.7}, 2);
    std::cout << "State space : " << *decision.getStateSpace() << std::endl;
    std::cout << "Action space : " << *decision.getActionSpace() << std::endl;
    std::cout << "Observation space : " << *decision.getObsSpace() << std::endl;
    std::cout << "Internal State : " << decision.getInternalState() << std::endl;
    std::cout << "Planning Horizon : " << decision.getPlanningHorizon() << std::endl;
    std::cout << "Discount factor : " << decision.getDiscount() << std::endl;
    std::cout << "Criterion : " << decision.getCriterion() << std::endl;
    std::cout << "Filename : " << decision.getFileName() << std::endl;
    std::cout << "Bound : " << decision.getBound() << std::endl;
    std::cout << "Transition : " << decision.getStateDynamics()->getTransitionProbability(0, 0, 0) << std::endl;

    std::cout << "Start Distrib : ";
    for (auto proba : decision.getStartDistrib().probabilities())
        std::cout << proba << "\t";
    std::cout << "\n\n";


    auto new_decision = decision.toMMDP();
    // DiscreteMDP decision(state_sp, action_sp, state_dyn, reward_f, {0.3, 0.7}, 2);
    std::cout << "State space : " << *new_decision->getStateSpace() << std::endl;
    std::cout << "Action space : " << *new_decision->getActionSpace() << std::endl;
    std::cout << "Observation space : " << *new_decision->getObsSpace() << std::endl;
    std::cout << "Internal State : " << new_decision->getInternalState() << std::endl;
    std::cout << "Planning Horizon : " << new_decision->getPlanningHorizon() << std::endl;
    std::cout << "Discount factor : " << new_decision->getDiscount() << std::endl;
    std::cout << "Criterion : " << new_decision->getCriterion() << std::endl;
    std::cout << "Filename : " << new_decision->getFileName() << std::endl;
    std::cout << "Bound : " << new_decision->getBound() << std::endl;
    std::cout << "Transition : " << new_decision->getStateDynamics()->getTransitionProbability(0, 0, 0) << std::endl;

    std::cout << "Start Distrib : ";
    for (auto proba : new_decision->getStartDistrib().probabilities())
        std::cout << proba << "\t";
    std::cout << "\n\n";



    auto new_decision2 = new_decision->toMDP();
    // DiscreteMDP decision(state_sp, action_sp, state_dyn, reward_f, {0.3, 0.7}, 2);
    std::cout << "State space : " << *new_decision2->getStateSpace() << std::endl;
    std::cout << "Action space : " << *new_decision2->getActionSpace() << std::endl;
    std::cout << "Observation space : " << *new_decision2->getObsSpace() << std::endl;
    std::cout << "Internal State : " << new_decision2->getInternalState() << std::endl;
    std::cout << "Planning Horizon : " << new_decision2->getPlanningHorizon() << std::endl;
    std::cout << "Discount factor : " << new_decision2->getDiscount() << std::endl;
    std::cout << "Criterion : " << new_decision2->getCriterion() << std::endl;
    std::cout << "Filename : " << new_decision2->getFileName() << std::endl;
    std::cout << "Bound : " << new_decision2->getBound() << std::endl;
    std::cout << "Transition : " << new_decision2->getStateDynamics()->getTransitionProbability(0, 0, 0) << std::endl;

    std::cout << "Start Distrib : ";
    for (auto proba : new_decision2->getStartDistrib().probabilities())
        std::cout << proba << "\t";
    std::cout << "\n\n";


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