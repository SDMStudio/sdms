#include <iostream>
#include <boost/program_options.hpp>

#include <sdm/types.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/algorithms.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;

int learn(int argv, char **args)
{
    try
    {
        std::string problem, algorithm, formalism, name, qvalue, q_init;
        unsigned long num_episodes, memory;
        number horizon, seed, batch_size;
        double lr, discount;
        bool compress, store_actions, store_states;

        po::options_description options("Options");
        options.add_options()("help", "produce help message")("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()("algorithm,a", po::value<string>(&algorithm)->default_value("qlearning"), "the learning algorithm to use")("problem,p", po::value<string>(&problem)->default_value("tiger"), "the problem to be solved")("formalism,f", po::value<string>(&formalism)->default_value("decpomdp"), "the formalism to use")("lr,l", po::value<double>(&lr)->default_value(0.01), "the learning rate")("discount,d", po::value<double>(&discount)->default_value(0.9), "the discount factor")("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")("num_episodes,t", po::value<unsigned long>(&num_episodes)->default_value(100000), "the maximum number of timesteps")("memory,m", po::value<unsigned long>(&memory)->default_value(0), "the memory for history")("seed,s", po::value<number>(&seed)->default_value(1), "the seed")("batch_size,b", po::value<number>(&batch_size)->default_value(0), "the seed")("compress", po::value<bool>(&compress)->default_value(true), "If true, apply compression when required.")("store_actions", po::value<bool>(&store_actions)->default_value(true), "If true, store the macro actions when required.")("store_states", po::value<bool>(&store_states)->default_value(true), "If true, store the macro states when required.")("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment");

        po::options_description algo_config("Algorithms configuration");
        algo_config.add_options()("qvalue,q", po::value<string>(&qvalue)->default_value("tabular"), "the representation of the Q-Value")("init,i", po::value<string>(&q_init)->default_value("ZeroInitializer"), "the Q-Value initialization method");

        po::options_description visible("\nUsage:\tsdms-solve [CONFIGS]\n\tSDMStudio solve [CONFIGS]\n\nSolve a problem with specified algorithms and configurations.");
        visible.add(options).add(config).add(algo_config);

        po::options_description config_file_options;
        config_file_options.add(config);

        po::variables_map vm;
        try
        {
            po::store(po::command_line_parser(argv, args).options(visible).run(), vm);
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

        // Build algorithm
        auto algo = sdm::algo::make(algorithm,
                               problem,
                               formalism,
                               horizon,
                               discount,
                               lr,
                               num_episodes,
                               1800,
                               name,
                               memory,
                               compress,
                               store_states,
                               store_actions,
                               batch_size,
                               qvalue);

        algo->initialize();
        algo->solve();

        if (vm.count("test"))
        {
            algo->test();
        }
    }
    catch (std::exception &e)
    {
        std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
        return sdm::ERROR_UNHANDLED_EXCEPTION;
    }

    return sdm::SUCCESS;
}

#ifndef __main_program__
#define __main_program__
int main(int argv, char **args)
{
    return learn(argv, args);
}
#endif