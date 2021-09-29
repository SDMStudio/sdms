#include <iostream>
#include <signal.h>
#include <cstdlib>
#include <boost/program_options.hpp>
#include <thread>
#include <atomic>
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
    std::cout << config::LOG_SDMS << "Exit process" << std::endl;
    algorithm->saveResults(name);
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
        std::string problem, algo_name, formalism, upper_bound, lower_bound, ub_init, lb_init, sawtooth_type_of_resolution, type_sawtooth_linear_programming;
        int trials, memory;
        number horizon, seed, batch_size, freq_update_lb, freq_update_ub;
        double error, discount;
        double p_b, p_o, p_c;
        bool compress, store_actions, store_states;
        number sawtooth_BigM_value;

        po::options_description options("Options");
        options.add_options()
        ("help", "produce help message")
        ("test", "test the policy found")
        ("save", "save the policy found");

        po::options_description config("Configuration");
        config.add_options()
        ("algorithm,a", po::value<string>(&algo_name)->default_value("hsvi"), "the algorithm to use")
        ("problem,p", po::value<string>(&problem)->default_value(sdm::config::PROBLEM_PATH + "dpomdp/mabc.dpomdp"), "the problem to be solved")
        ("formalism,f", po::value<string>(&formalism)->default_value("DecPOMDP"), "the formalism to use")
        ("error,e", po::value<double>(&error)->default_value(0.001), "the error")
        ("discount,d", po::value<double>(&discount)->default_value(1.0), "the discount factor")
        ("horizon,h", po::value<number>(&horizon)->default_value(5), "the planning horizon")
        ("trials,t", po::value<int>(&trials)->default_value(100000), "the maximum number of trials")
        ("memory,m", po::value<int>(&memory)->default_value(-1), "the memory for history")
        ("seed,s", po::value<number>(&seed)->default_value(1), "the seed")
        ("batch_size,b", po::value<number>(&batch_size)->default_value(0), "the batch size used in learning algorithms")
        ("compress", po::value<bool>(&compress)->default_value(true), "If true, apply compression when required.")
        ("store_states", po::value<bool>(&store_states)->default_value(true), "If true, store the macro states when required.")
        ("store_actions", po::value<bool>(&store_actions)->default_value(true), "If true, store the macro actions when required.")
        ("p_c", po::value<double>(&p_c)->default_value(config::PRECISION_COMPRESSION), "The precision of the compression.")
        ("p_b", po::value<double>(&p_b)->default_value(config::PRECISION_BELIEF), "The precision of beliefs.")
        ("p_o", po::value<double>(&p_o)->default_value(config::PRECISION_OCCUPANCY_STATE), "The precision of occupancy states.")
        ("time_max", po::value<double>(&MAX_RUNNING_TIME)->default_value(10000), "The maximum running time.")
        ("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment");

        po::options_description algo_config("Algorithms configuration");
        algo_config.add_options()
        ("lower_bound", po::value<string>(&lower_bound)->default_value("tabular"), "the lower bound representation (HSVI, ValueIteration)")
        ("upper_bound", po::value<string>(&upper_bound)->default_value("tabular"), "the upper bound representation (HSVI)")
        ("lb_init", po::value<string>(&lb_init)->default_value("Min"), "the lower bound initialization method (HSVI, ValueIteration)")
        ("ub_init", po::value<string>(&ub_init)->default_value("Max"), "the upper bound initialization method (HSVI)")
        ("freq_update_lb", po::value<number>(&freq_update_lb)->default_value(1), "the batch size used in learning algorithms")
        ("freq_update_ub", po::value<number>(&freq_update_ub)->default_value(1), "the batch size used in learning algorithms")
        ("sawtooth_type_of_resolution", po::value<string>(&sawtooth_type_of_resolution)->default_value("IloIfThen"), "the type of resolution used for sawtooth (BigM, IloIfThen)")
        ("sawtooth_BigM_value", po::value<number>(&sawtooth_BigM_value)->default_value(100), "the upper bound initialization method (HSVI)")
        ("type_sawtooth_linear_programming", po::value<string>(&type_sawtooth_linear_programming)->default_value("Full"), "the type of linear program used for sawtooth (Relaxed, Full)");

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

        // Set the seed
        common::global_urng().seed(seed);

        // Set precisions
        Belief::PRECISION = p_b;
        OccupancyState::PRECISION = p_o;
        PrivateOccupancyState::PRECISION_COMPRESSION = p_c;

        // Build algorithm
        algorithm = sdm::algo::make(algo_name,
                                    problem,
                                    formalism,
                                    upper_bound,
                                    lower_bound,
                                    ub_init,
                                    lb_init,
                                    discount,
                                    error,
                                    horizon,
                                    trials,
                                    memory,
                                    compress,
                                    store_actions,
                                    store_states,
                                    name,
                                    MAX_RUNNING_TIME,
                                    freq_update_lb,
                                    freq_update_ub,
                                    sawtooth_type_of_resolution,
                                    sawtooth_BigM_value,
                                    type_sawtooth_linear_programming, PAIRWISE, -1, NONE, -1,
                                    batch_size);

        // Initialize algorithm
        algorithm->do_initialize();

        // Launch a thread that will sleep during the max time allowed and kill the process afterward
        std::thread thread_sleep(thread_sdms_sleep);

        // Solve the problem
        algorithm->do_solve();
        quit_flag = true; // pogram ending; set the quit flag
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        algorithm->saveResults(name);

        thread_sleep.join();

        if (vm.count("test"))
        {
            algorithm->do_test();
        }
        if (vm.count("save"))
        {
            algorithm->do_save();
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