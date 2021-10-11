#include <iostream>

#include <boost/program_options.hpp>

#include <sdm/types.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/algorithms/q_learning.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/utils/value_function/pwlc_qvalue_function.hpp>
#include <sdm/utils/rl/exploration.hpp>
#include <sdm/utils/value_function/backup/tabular_qvalue_backup.hpp>
#include <sdm/utils/rl/experience_memory.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/core/state/private_occupancy_state.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;
int main(int argc, char **argv)
{
    try
    {
        std::string path, formalism, name, qvalue, q_init;
        unsigned long max_steps;
        number horizon, memory, batch_size;
        double lr, discount, sf, belief_precision, ostate_precision, compress_precision;
        int seed;

        po::options_description options("Options");
        options.add_options()("help", "produce help message")("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()("path,p", po::value<string>(&path)->default_value("tiger"), "the path to the problem to be solved")("formalism,f", po::value<string>(&formalism)->default_value("MDP"), "the formalism to use e.g. MDP, MMDP, POMDP, MPOMDP")("lr,l", po::value<double>(&lr)->default_value(0.01), "the learning rate")("smooth,a", po::value<double>(&sf)->default_value(0.999), "the smoothing factor for the E[R]")("belief_precision", po::value<double>(&belief_precision)->default_value(0.001), "the precision of beliefs")("ostate_precision", po::value<double>(&ostate_precision)->default_value(0.1), "the precision of occupancy states")("compress_precision", po::value<double>(&compress_precision)->default_value(0.1), "the precision of the compression")("discount,d", po::value<double>(&discount)->default_value(1.0), "the discount factor")("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")("memory,m", po::value<number>(&memory)->default_value(-1), "the memory. If 0 then infinite memory.")("batch_size,b", po::value<number>(&batch_size)->default_value(0), "batch size, that is K from the paper")("max_steps,t", po::value<unsigned long>(&max_steps)->default_value(100000), "the maximum number of timesteps")("seed,s", po::value<int>(&seed)->default_value(1), "random seed")("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment");

        po::options_description algo_config("Algorithms configuration");
        algo_config.add_options()("qvalue,q", po::value<string>(&qvalue)->default_value("tabular"), "the representation of the Q-Value")("init,i", po::value<string>(&q_init)->default_value("ZeroInitializer"), "the Q-Value initialization method");

        po::options_description visible("\nUsage:\tsdms-solve [CONFIGS]\n\tSDMStudio solve [CONFIGS]\n\nSolve a path with specified algorithms and configurations.");
        visible.add(options).add(config).add(algo_config);

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

        common::global_urng().seed(seed);

        auto dpomdp = sdm::parser::parse_file(path);

        std::cout << *dpomdp << std::endl;

        for (auto state : *dpomdp->getStateSpace(0))
        {
            for (auto action : *dpomdp->getActionSpace(0))
            {
                for (auto next_state : dpomdp->getReachableStates(state->toState(), action->toAction(), 0))
                {
                    std::cout << "P(" << state->str() << ", " << action->str() << ", " << next_state->str() << ") = " << dpomdp->getStateDynamics()->getTransitionProbability(state->toState(), action->toAction(), next_state->toState(), 0) << " = "<< dpomdp->getStateDynamics()->getNextStateDistribution(state->toState(), action->toAction(), 0)->getProbability(next_state->toState(), nullptr) << std::endl;
                }
            }
        }
    }
    catch (std::exception &e)
    {
        std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
        return sdm::ERROR_UNHANDLED_EXCEPTION;
    }

    return sdm::SUCCESS;

} // END main
