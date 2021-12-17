#include <iostream>
#include <signal.h>
#include <cstdlib>
#include <boost/program_options.hpp>
#include <thread>
#include <atomic>
#include <experimental/filesystem>
#include <chrono>

#include <sdm/types.hpp>
#include <sdm/config.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/worlds.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;

double MAX_RUNNING_TIME = 1800;
bool quit_flag = false;
std::string name = "result";
std::shared_ptr<Algorithm> algorithm;

void leave()
{
    algorithm->logging();
    std::cout << config::LOG_SDMS << "Exit process" << std::endl;
    algorithm = nullptr;
    exit(0);
}

void handler(int)
{
    leave();
}

void thread_sdms_sleep()
{
    unsigned int interval_millisecs = 100, total_time_millisecs = 0, max_time_millisecs = MAX_RUNNING_TIME * 1000;
    while ((!quit_flag) && (total_time_millisecs < max_time_millisecs))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_millisecs));
        total_time_millisecs += interval_millisecs;
    }
    if (total_time_millisecs >= max_time_millisecs)
    {
        std::cout << config::LOG_SDMS << "Out of time : MAX_RUNNING_TIME=" << MAX_RUNNING_TIME << std::endl;
        raise(SIGINT);
    }
}

int solve(int argv, char **args)
{
    // Handle interrupt signal
    signal(SIGINT, &handler);

    try
    {
        std::string world, algo_name, formalism, upper_bound, lower_bound, ub_init, lb_init, type_sampling;
        int trials, memory;
        number horizon, seed, batch_size, freq_update_lb, freq_update_ub, state_type;
        double error, discount;
        double p_b, p_o, p_c;
        bool store_actions, store_states;
        unsigned long long num_samples;

        int freq_pruning_v1 = 1, freq_pruning_v2 = 1;
        std::string type_of_resolution_v1, type_of_resolution_v2, type_of_pruning_v1, type_of_pruning_v2;

        po::options_description options("Options");
        options.add_options()("help", "produce help message")("test", "test the policy found")("save", "save the policy found");

        po::options_description config("Configuration");
        config.add_options()("algorithm,a", po::value<string>(&algo_name)->default_value("hsvi"), "the algorithm to use")("world,w", po::value<string>(&world)->default_value("mabc.dpomdp"), "the world to be solved")("formalism,f", po::value<string>(&formalism)->default_value("OccupancyMDP"), "the formalism to use")("horizon,h", po::value<number>(&horizon)->default_value(5), "the planning horizon")("discount,d", po::value<double>(&discount)->default_value(1.0), "the discount factor")("memory,m", po::value<int>(&memory)->default_value(-1), "the memory for history")("error,e", po::value<double>(&error)->default_value(0.001), "the error")("trials,t", po::value<int>(&trials)->default_value(100000), "the maximum number of trials")("seed,s", po::value<number>(&seed)->default_value(1), "the seed")("batch_size,b", po::value<number>(&batch_size)->default_value(0), "the batch size used in rl")("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment")("compression", po::value<number>(&state_type)->default_value(0), "The type of occupancy state (COMPRESSED, ONE_STEP, UNCOMPRESSED)")("store_states", po::value<bool>(&store_states)->default_value(true), "If true, store the macro states.")("store_actions", po::value<bool>(&store_actions)->default_value(true), "If true, store the macro actions.")("p_c", po::value<double>(&p_c)->default_value(config::PRECISION_COMPRESSION), "The precision of the compression.")("p_b", po::value<double>(&p_b)->default_value(config::PRECISION_BELIEF), "The precision of beliefs.")("p_o", po::value<double>(&p_o)->default_value(config::PRECISION_OCCUPANCY_STATE), "The precision of occupancy states.")("time_max", po::value<double>(&MAX_RUNNING_TIME)->default_value(1800), "The maximum running time.");

        po::options_description hsvi_config("HSVI configuration");
        hsvi_config.add_options()("lower_bound", po::value<string>(&lower_bound)->default_value("tabular"), "the lower bound representation")("upper_bound", po::value<string>(&upper_bound)->default_value("tabular"), "the upper bound representation")("lb_init", po::value<string>(&lb_init)->default_value("Min"), "the lower bound initialization method")("ub_init", po::value<string>(&ub_init)->default_value("Max"), "the upper bound initialization method")("freq_update_lb", po::value<number>(&freq_update_lb)->default_value(1), "the update frequency of the lower bound.")("freq_update_ub", po::value<number>(&freq_update_ub)->default_value(1), "the update frequency of the upper bound.")("lb_type_of_resolution", po::value<string>(&type_of_resolution_v1)->default_value("IloIfThen"), "the type of resolution for the lower bound (ex: 'BigM:100' or 'IloIfThen' for LP)")("ub_type_of_resolution", po::value<string>(&type_of_resolution_v2)->default_value("IloIfThen"), "the type of resolution for the upper bound (ex: 'BigM:100' or 'IloIfThen' for LP)")("lb_freq_pruning", po::value<int>(&freq_pruning_v1)->default_value(1), "the pruning frequency for the first value function.")("ub_freq_pruning", po::value<int>(&freq_pruning_v2)->default_value(1), "the pruning frequency for the second value function .")("lb_type_of_pruning", po::value<string>(&type_of_pruning_v1)->default_value("none"), "the pruning type for the lower bound (ex: 'bounded', 'pairwise', 'none'")("ub_type_of_pruning", po::value<string>(&type_of_pruning_v2)->default_value("none"), "the pruning type for the upper bound (ex: 'iterative', 'global', 'none'");

        po::options_description pbvi_config("PBVI configuration");
        pbvi_config.add_options()("num_samples", po::value<unsigned long long>(&num_samples)->default_value(10), "the number of sample to generate in the algorithm")("type_sampling", po::value<string>(&type_sampling)->default_value(""), "the type of sampling process")("value_function", po::value<string>(&lower_bound), "the lower bound representation")("vf_init", po::value<string>(&lb_init), "the lower bound initialization method")("freq_update", po::value<number>(&freq_update_lb), "the update frequency of the lower bound.")("type_of_resolution", po::value<string>(&type_of_resolution_v1), "the type of resolution for the lower bound (ex: 'BigM:100' or 'IloIfThen' for LP)")("freq_pruning", po::value<int>(&freq_pruning_v1), "the pruning frequency for the first value function.")("type_of_pruning", po::value<string>(&type_of_pruning_v1), "the pruning type for the lower bound (ex: 'bounded', 'pairwise', 'none'");

        po::options_description qlearning_config("Q-learning configuration");
        qlearning_config.add_options()("qvalue_function", po::value<string>(&lower_bound), "the q-value function representation")("lr", po::value<double>(&error), "the leraning rate")("q_init", po::value<string>(&lb_init), "the q-value function initialization method");

        po::options_description byg_config("Bayesian game solver configuration");
        byg_config.add_options()("player_id", po::value<number>(&batch_size), "the identifier of player");

        po::options_description visible("\nUsage:\tsdms-solve [CONFIGS]\n\tSDMStudio solve [CONFIGS]\n\nSolve a problem with specified algorithms and configurations.");
        visible.add(options).add(config).add(hsvi_config).add(pbvi_config).add(qlearning_config).add(byg_config);

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
            if (discount >= 1 && !horizon)
            {
                std::cerr << config::LOG_SDMS << "Invalid hyperparameters (discount must be less than 1. for infinite horizon)" << std::endl;
                return sdm::ERROR_IN_COMMAND_LINE;
            }
        }
        catch (po::error &e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl;
            std::cerr << visible << std::endl;
            return sdm::ERROR_IN_COMMAND_LINE;
        }

        // Set the seed
        common::global_urng().seed(seed);

        // Set precisions
        Belief::PRECISION = p_b;
        OccupancyState::PRECISION = p_o;
        PrivateOccupancyState::PRECISION_COMPRESSION = p_c;

        common::logo();

        // Build algorithm
        algorithm = sdm::algo::make(algo_name,
                                    world,
                                    formalism,
                                    horizon,
                                    discount,
                                    error,
                                    trials,
                                    MAX_RUNNING_TIME,
                                    name,
                                    memory,
                                    (StateType)state_type,
                                    store_states,
                                    store_actions,
                                    batch_size,
                                    num_samples,
                                    type_sampling,
                                    lower_bound,
                                    lb_init,
                                    freq_update_lb,
                                    type_of_resolution_v1,
                                    freq_pruning_v1,
                                    type_of_pruning_v1,
                                    upper_bound,
                                    ub_init,
                                    freq_update_ub,
                                    type_of_resolution_v2,
                                    freq_pruning_v2,
                                    type_of_pruning_v2);

        // Initialize algorithm
        algorithm->initialize();

        // Launch a thread that will sleep during the max time allowed and kill the process afterward
        std::thread thread_sleep(thread_sdms_sleep);

        // Solve the problem
        algorithm->solve();

        quit_flag = true; // pogram ending; set the quit flag
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // algorithm->saveResults(name);

        thread_sleep.join();

        if (vm.count("test"))
        {
            algorithm->test();
        }
        if (vm.count("save"))
        {
            algorithm->save();
        }
    }
    catch (std::exception &e)
    {
        std::cerr << "Unhandled Exception reached the top of main: " << e.what() << std::endl;
        return sdm::ERROR_UNHANDLED_EXCEPTION;
    }

    return sdm::SUCCESS;
}

#ifndef __main_program__
#define __main_program__
int main(int argv, char **args)
{

    // return solve(argv, args);
    return solve(argv, args);
}
#endif