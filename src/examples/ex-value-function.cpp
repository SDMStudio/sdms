
#include <cstdlib>
#include <iostream>

#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>
#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>
#include <sdm/utils/value_function/update_rule/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename = (argc > 1) ? argv[1] : config::PROBLEM_PATH + "dpomdp/mabc.dpomdp";
    number horizon = 10, truncation = 1;
    double error = 0.00001, discount = 1.;
    try
    {
        // Parse file into MPOMDP
        auto mdp = sdm::parser::parse_file(filename);
        mdp->setHorizon(horizon);
        mdp->setDiscount(discount);

        // Instanciate the problem
        std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<OccupancyMDP>(mdp, (truncation > 0) ? truncation : horizon, true, true);

        auto lb_init = std::make_shared<MinInitializer>(hsvi_mdp);
        auto action_tabular = std::make_shared<ExhaustiveActionSelection>(hsvi_mdp);
        auto value = std::make_shared<TabularValueFunction>(hsvi_mdp, lb_init, action_tabular);

        std::cout << *value << std::endl;

        for (auto state : *hsvi_mdp->getUnderlyingProblem()->getStateSpace(0))
        {
            value->setValueAt(state, 10., 0);
        }
        std::cout << *value << std::endl;

        value->save("TabVal.bin");

        std::cout << "pass" << std ::endl;
        auto value2 = std::make_shared<TabularValueFunction>(hsvi_mdp, lb_init, action_tabular);
        std::cout << "pass1" << std ::endl;

        value2->load("TabVal.bin");

        std::cout << *value2 << std::endl;
    }
    catch (exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }

    return 0;
} // END main