
#include <iostream>

#include <memory>
#include <sdm/exception.hpp>

// #include <sdm/algorithms/hsvi.hpp>

#include <sdm/utils/struct/graph.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/base_item.hpp>
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
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/history_tree.hpp>
#include <sdm/core/state/jhistory_tree.hpp>
#include <sdm/core/state/occupancy_state.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename;
    number horizon = 4, truncation;
    double discount = 1.0, error = 0.01, trial = 1000;

    if (argc > 1)
    {
        filename = argv[1];

        if (argc > 2)
        {
            horizon = std::stoi(argv[2]);
            truncation = horizon;
            if (argc > 2)
            {
                truncation = std::stoi(argv[3]);
            }
        }
    }
    else
    {
        std::cerr << "Error: Require 1 input file." << std::endl;
        return 1;
    }

    try
    {

        // auto graph = std::make_shared<Graph<int, char>>();

        // graph->addNode(0);
        // graph->addNode(1);
        // graph->addNode(2);
        // graph->addNode(3);
        // graph->addNode(4);

        // graph->getNode(2)->addSuccessor('c', 3);
        // graph->getNode(4)->addSuccessor('c', 3);
        // graph->getNode(4)->addSuccessor('a', 3);

        // std::cout << *graph << std::endl;

        // std::cout << graph->contains(2) << std::endl;
        // std::cout << graph->contains(4) << std::endl;
        // std::cout << graph->contains(42) << std::endl;

        std::cout << "# Parse File" << std::endl;
        auto mdp = sdm::parser::parse_file(filename);
        mdp->setHorizon(horizon);
        mdp->setDiscount(discount);

        std::cout << mdp->toStdFormat() << std::endl;

        std::cout << "# Build OccupancyMDP" << std::endl;
        std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<OccupancyMDP>(mdp, truncation);

        auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);
        auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi_mdp);

        auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
        auto init_ub = std::make_shared<MaxInitializer>(hsvi_mdp);

        auto lb = std::make_shared<TabularValueFunction>(mdp->getHorizon(), init_lb, tabular_backup, action_tabular);
        auto ub = std::make_shared<TabularValueFunction>(mdp->getHorizon(), init_ub, tabular_backup, action_tabular);
        auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, mdp->getHorizon(), error, trial);

        std::cout << "# Initialize Algo" << std::endl;
        algo->do_initialize();
        std::cout << *algo->getLowerBound() << std::endl;
        std::cout << *algo->getUpperBound() << std::endl;
        std::cout << "# Solve Algo" << std::endl;
        algo->do_solve();
        std::cout << "# Solved" << std::endl;

        std::cout << *algo->getLowerBound() << std::endl;
        std::cout << *algo->getUpperBound() << std::endl;
    }
    catch (exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }

} // END main
