
#include <iomanip>
#include <iostream>

#include <boost/program_options.hpp>

#include <memory>
#include <sdm/config.hpp>
#include <sdm/exception.hpp>

#include <sdm/algorithms.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/base_item.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>

#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/hierarchical_occupancy_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/world/serial_mpomdp.hpp>
#include <sdm/world/hierarchical_mpomdp.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>
#include <sdm/utils/value_function/action_selection/action_maxplan.hpp>
#include <sdm/utils/value_function/action_selection/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/action_selection/action_sawtooth_lp_serial.hpp>
#include <sdm/utils/value_function/action_selection/action_maxplan_serial.hpp>
#include <sdm/utils/value_function/action_selection/action_maxplan_lp.hpp>

#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/vfunction/point_set_value_function.hpp>
#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>

#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/algorithms.hpp>

using namespace sdm;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
    try
    {
        std::string path;
        unsigned long n;
        bool dyn_cast;

        po::options_description options("Options");
        options.add_options()("help", "produce help message")("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()("path,p", po::value<std::string>(&path)->default_value("tiger"), "the path to the problem to be solved")("dynamic_cast,d", po::value<bool>(&dyn_cast)->default_value(true), "If true, store the macro actions when required.")("num_cast,n", po::value<unsigned long>(&n)->default_value(10000), "the frequency for updating the upper bound");

        po::options_description visible("\nUsage:\tsdms-solve [CONFIGS]\n\tSDMStudio solve [CONFIGS]\n\nSolve a path with specified algorithms and configurations.");
        visible.add(options).add(config);

        po::options_description config_file_options;
        config_file_options.add(config);

        po::variables_map vm;
        try
        {
            po::store(po::command_line_parser(argc, argv).options(visible).run(), vm);
            po::notify(vm);
            if (vm.count("help"))
            {
                std::cout << visible << std::endl;
                return sdm::SUCCESS;
            }
        }
        catch (po::error &e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl;
            std::cerr << visible << std::endl;
            return sdm::ERROR_IN_COMMAND_LINE;
        }

        // Parse file into MPOMDP
        auto mdp = sdm::parser::parse_file(path);
        std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<OccupancyMDP>(mdp);

        std::cout << "Num CAST : "<< n << std::endl;

        if (dyn_cast)
        {
            for (unsigned long i = 0; i < n; i++)
                auto ptr = std::dynamic_pointer_cast<OccupancyMDP>(hsvi_mdp);
        }
        else
        {
            for (unsigned long i = 0; i < n; i++)
                auto ptr = std::static_pointer_cast<OccupancyMDP>(hsvi_mdp);
        }
    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }

} // END main
