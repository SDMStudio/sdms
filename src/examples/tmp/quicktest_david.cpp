#include <iostream>

#include <memory>
#include <sdm/exception.hpp>

// #include <sdm/algorithms/hsvi.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>

#include <sdm/world/mdp.hpp>
#include <sdm/world/mmdp.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/parser/parser.hpp>

// #include <sdm/utils/value_function/tabular_value_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    number horizon = 3;

    auto mdp = sdm::parser::parse_file("../data/world/dpomdp/mabc.dpomdp");
    mdp->setHorizon(horizon);

    std::cout << mdp->toStdFormat() << std::endl;

    // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<BeliefMDP>(mdp);
    // std::cout << "1" << std::endl;

    // auto lb = std::make_shared<MappedValueFunction>(hsvi_mdp, mdp->getHorizon(), -1000);
    // auto ub = std::make_shared<MappedValueFunction>(hsvi_mdp, mdp->getHorizon(), 1000);
    // std::cout << "2" << std::endl;

    // auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, mdp->getHorizon(), 0.01);
    // std::cout << "3" << std::endl;
    // algo->do_initialize();
    // std::cout << *algo->getLowerBound() << std::endl;
    // std::cout << *algo->getUpperBound() << std::endl;
    // std::cout << "4" << std::endl;
    // algo->do_solve();
    // std::cout << *algo->getLowerBound() << std::endl;
    // std::cout << *algo->getUpperBound() << std::endl;

} // END main
