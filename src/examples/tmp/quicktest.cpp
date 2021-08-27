#include <iostream>

#include <memory>
#include <sdm/exception.hpp>

#include <sdm/parser/parser.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/algorithms/hsvi.hpp>

#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    number horizon = 5;

    auto mdp_tiger = parser::parse_file("../data/world/dpomdp/tiger.dpomdp");

    mdp_tiger->setHorizon(horizon);

    // Creation of HSVI problem and Resolution
    std::shared_ptr<SolvableByHSVI> beliefMDP = std::make_shared<BeliefMDP>(mdp_tiger);

    auto tabular_backup = std::make_shared<TabularBackup>(beliefMDP);
    auto tabular_action = std::make_shared<ActionVFTabulaire>(beliefMDP);

    auto init_lb = std::make_shared<MinInitializer>(beliefMDP);
    auto init_ub = std::make_shared<MaxInitializer>(beliefMDP);

    auto lb = std::make_shared<TabularValueFunction>(horizon, init_lb, tabular_backup, tabular_action, false);
    auto ub = std::make_shared<TabularValueFunction>(horizon, init_ub, tabular_backup, tabular_action, true);

    auto algo = std::make_shared<HSVI>(beliefMDP, lb, ub, horizon, 0.01);
    algo->do_initialize();
    std::cout << *algo->getLowerBound() << std::endl;
    std::cout << *algo->getUpperBound() << std::endl;
    algo->do_solve();

    return 0;
}
