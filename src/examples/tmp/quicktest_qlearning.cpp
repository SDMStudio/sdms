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
#include <sdm/utils/rl/exploration.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;

int learn(int argv, char **args)
{
    try
    {
        std::string path, formalism, name, qvalue, q_init;
        unsigned long max_steps;
        number horizon, memory, batch_size;
        double lr, discount, sf, precision, precision2;
        int seed;

        po::options_description options("Options");
        options.add_options()
        ("help", "produce help message")
        ("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()
        ("path,p", po::value<string>(&path)->default_value("tiger"), "the path to the problem to be solved")
        ("formalism,f", po::value<string>(&formalism)->default_value("MDP"), "the formalism to use e.g. MDP, MMDP, POMDP, MPOMDP")
        ("lr,l", po::value<double>(&lr)->default_value(0.01), "the learning rate")
        ("smooth,a", po::value<double>(&sf)->default_value(0.999), "the smoothing factor for the E[R]")
        ("precision,r", po::value<double>(&precision)->default_value(0.000001), "the precision of hierarchical private occupancy states (occupancy states) ")
        ("precision2,w", po::value<double>(&precision2)->default_value(0.000001), "the precision of private occupancy states (private occupancy states)")
        ("discount,d", po::value<double>(&discount)->default_value(1.0), "the discount factor")
        ("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")
        ("memory,m", po::value<number>(&memory)->default_value(0), "the memory. If 0 then infinite memory.")
        ("batch_size,b", po::value<number>(&batch_size)->default_value(10), "batch size, that is K from the paper")
        ("max_steps,t", po::value<unsigned long>(&max_steps)->default_value(100000), "the maximum number of timesteps")
        ("seed,s", po::value<int>(&seed)->default_value(1), "random seed")
        ("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment");

        po::options_description algo_config("Algorithms configuration");
        algo_config.add_options()
        ("qvalue,q", po::value<string>(&qvalue)->default_value("tabular"), "the representation of the Q-Value")
        ("init,i", po::value<string>(&q_init)->default_value("ZeroInitializer"), "the Q-Value initialization method");

        po::options_description visible("\nUsage:\tsdms-solve [CONFIGS]\n\tSDMStudio solve [CONFIGS]\n\nSolve a path with specified algorithms and configurations.");
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

        // auto dpomdp = sdm::parser::parse_file("../data/world/dpomdp/boxPushingUAI07.dpomdp");
        // auto dpomdp = sdm::parser::parse_file("../data/world/dpomdp/Grid3x3corners.dpomdp");
        auto dpomdp = sdm::parser::parse_file(path);

        auto state_space = dpomdp->getStateSpace();
        auto action_space = dpomdp->getActionSpace();
        auto reward_space = dpomdp->getReward(); 
        auto state_dynamics = dpomdp->getStateDynamics();

        auto start_distribution = dpomdp->getStartDistribution();
        auto observation_space = dpomdp->getObservationSpace(0);
        auto observation_dynamics = dpomdp->getObservationDynamics();

        std::shared_ptr<GymInterface> gym;

        if (formalism == "MDP")
        {
            gym = std::make_shared<MDP>(state_space, action_space, reward_space, state_dynamics, start_distribution, horizon, 1.);
        }
        else if (formalism == "MMDP")
        {
            gym = std::make_shared<MMDP>(state_space, action_space, reward_space, state_dynamics, start_distribution, horizon, 1.);
        }
        else if (formalism == "POMDP")
        {
            gym = std::make_shared<POMDP>(state_space, action_space, observation_space, reward_space, state_dynamics, observation_dynamics, start_distribution, horizon, 1.);
        }
        else if (formalism == "MPOMDP")
        {
            gym = std::make_shared<MPOMDP>(state_space, action_space, observation_space, reward_space, state_dynamics, observation_dynamics, start_distribution, horizon, 1.);
        }
        else
        {
            // error
        }

        std::shared_ptr<ZeroInitializer> initializer = std::make_shared<sdm::ZeroInitializer>();

        std::shared_ptr<QValueFunction> q_table = std::make_shared<TabularQValueFunction>(horizon, lr, initializer);

        std::shared_ptr<ZeroInitializer> target_initializer = std::make_shared<sdm::ZeroInitializer>();

        std::shared_ptr<QValueFunction> target_q_table = std::make_shared<TabularQValueFunction>(horizon, lr, target_initializer);

        std::shared_ptr<EpsGreedy> exploration = std::make_shared<EpsGreedy>();

        std::shared_ptr<Algorithm> algorithm = std::make_shared<QLearning>(gym, q_table, target_q_table, exploration, horizon, discount, lr, 1, max_steps);

        algorithm->do_initialize();

        algorithm->do_solve();
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