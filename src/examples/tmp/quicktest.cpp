#include <iostream>
#include <cassert>

#include <typeinfo>
#include <sdm/common.hpp>
#include <sdm/world/base/stochastic_process_base.hpp>
#include <sdm/world/base/po_process_base.hpp>
#include <sdm/world/decision_process.hpp>
#include <sdm/world/po_decision_process.hpp>

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

    // using DecisionProcessType = PartiallyObservableDecisionProcess<DiscreteSpace, MultiDiscreteSpace, MultiDiscreteSpace, StateDynamics, ObservationDynamics, std::vector<Reward>, std::discrete_distribution<number>>;

    auto state_sp = std::make_shared<DiscreteSpace>(2);
    auto action_sp = std::make_shared<MultiDiscreteSpace>(std::vector<number>{2, 2, 2});
    auto obs_sp = std::make_shared<MultiDiscreteSpace>(std::vector<number>{8, 2, 3});
    auto state_dyn = std::make_shared<StateDynamics>(state_sp->getNumItems(), action_sp->getNumItems());
    auto obs_dyn = std::make_shared<ObservationDynamics>(action_sp->getNumItems(), obs_sp->getNumItems(), state_sp->getNumItems());

    std::cout << "1" << std::endl;
    std::cout << action_sp->getNumSpaces() << std::endl;
    std::cout << "2" << std::endl;
    std::shared_ptr<std::vector<Reward>> rews_fct = std::make_shared<std::vector<Reward>>();
    for (int i = 0; i < action_sp->getNumSpaces(); i++)
    {
        rews_fct->push_back(Reward(state_sp->getNumItems(), action_sp->getNumItems()));
    }
    auto reward_f = std::make_shared<Reward>(state_sp->getNumItems(), action_sp->getNumItems());

    DiscretePOSG decision(state_sp, action_sp, obs_sp, state_dyn, obs_dyn, rews_fct, {0.3, 0.7});
    std::cout << *decision.getStateSpace() << std::endl;
    std::cout << *decision.getActionSpace() << std::endl;
    std::cout << *decision.getObsSpace() << std::endl;
    std::cout << decision.getInternalState() << std::endl;
    std::cout << decision.getPlanningHorizon() << std::endl;
    std::cout << decision.getDiscount() << std::endl;
    std::cout << decision.getCriterion() << std::endl;
    std::cout << decision.getFileName() << std::endl;
    std::cout << decision.getBound() << std::endl;
    // std::cout << decision.get() << std::endl;
    std::cout << "Transition " << decision.getStateDynamics()->getTransitionProbability(0, 0, 0) << std::endl;
    std::cout << "MaxReward " << (*decision.getReward())[0].getMaxReward() << std::endl;
    std::cout << "MinReward " << (*decision.getReward())[0] << std::endl;
    std::cout << "MinReward " << (*decision.getReward())[1] << std::endl;

    for (auto proba : decision.getStartDistrib().probabilities())
        std::cout << proba << "\t";

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
