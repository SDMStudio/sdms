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
#include <sdm/utils/value_function/hierarchical_qvalue_function.hpp>
#include <sdm/utils/rl/exploration.hpp>
#include <sdm/utils/value_function/backup/tabular_qvalue_backup.hpp>
#include <sdm/utils/value_function/backup/simple_hierarchical_qvalue_backup.hpp>
#include <sdm/utils/value_function/backup/hierarchical_qvalue_backup.hpp>
#include <sdm/utils/rl/experience_memory.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/private_hierarchical_occupancy_mdp.hpp>
#include <sdm/world/private_hierarchical_occupancy_mdp_with_history.hpp>

#include <sdm/core/state/private_occupancy_state.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;
int main(int argc, char **argv)
{
    try
    {
        std::string path, formalism, name, qvalue, version, q_init;
        unsigned long max_steps;
        number horizon, memory, batch_size;
        double lr, discount, sf, p_b, p_o, p_c, ball_r;
        int seed;
        bool store_actions, store_action_spaces;

        po::options_description options("Options");
        options.add_options()
        ("help", "produce help message")("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()
        ("path,p", po::value<string>(&path)->default_value("tiger"), "the path to the problem to be solved")
        ("formalism,f", po::value<string>(&formalism)->default_value("MDP"), "the formalism to use e.g. MDP, MMDP, POMDP, MPOMDP")
        ("lr,l", po::value<double>(&lr)->default_value(0.01), "the learning rate")
        ("smooth,a", po::value<double>(&sf)->default_value(0.999), "the smoothing factor for the E[R]")
        ("p_b", po::value<double>(&p_b)->default_value(0.0001), "the precision of belief state")
        ("p_o", po::value<double>(&p_o)->default_value(0.0001), "the precision of occupancy state ")
        ("p_c", po::value<double>(&p_c)->default_value(0.01), "the precision of compression ")
        ("ball_r", po::value<double>(&ball_r)->default_value(1.0), "the radius of the balls of s in the hqvf")
        ("discount,d", po::value<double>(&discount)->default_value(1.0), "the discount factor")
        ("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")
        ("memory,m", po::value<number>(&memory)->default_value(-1), "the memory. If 0 then infinite memory.")
        ("batch_size,b", po::value<number>(&batch_size)->default_value(0), "batch size, that is K from the paper")
        ("max_steps,t", po::value<unsigned long>(&max_steps)->default_value(100000), "the maximum number of timesteps")
        ("seed,s", po::value<int>(&seed)->default_value(1), "random seed")
        ("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment")
        ("store_actions", "store actions")("store_action_spaces", "store action spaces")
        ;

        po::options_description algo_config("Algorithms configuration");
        algo_config.add_options()
        ("qvalue,q", po::value<string>(&qvalue)->default_value("tabular"), "the representation of the Q-Value")
        ("version,v", po::value<string>(&version)->default_value("1"), "the version of hierarchical qvf")
        ("init,i", po::value<string>(&q_init)->default_value("ZeroInitializer"), "the Q-Value initialization method");

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

            if (vm.count("store_actions"))
                store_actions = true;
            else
                store_actions = false;
            
            if (vm.count("store_action_spaces"))
                store_action_spaces = true;
            else
                store_action_spaces = false;
            
        }
        catch (po::error &e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl;
            std::cerr << visible << std::endl;
            return sdm::ERROR_IN_COMMAND_LINE;
        }

        common::global_urng().seed(seed);

        auto dpomdp = sdm::parser::parse_file(path);
        dpomdp->setHorizon(horizon);
        dpomdp->setDiscount(discount);

        auto start_distribution = dpomdp->getStartDistribution();

        auto state_space = dpomdp->getStateSpace();
        auto action_space = dpomdp->getActionSpace();
        auto observation_space = dpomdp->getObservationSpace(0);
        auto reward_space = dpomdp->getRewardSpace();

        auto state_dynamics = dpomdp->getStateDynamics();
        auto observation_dynamics = dpomdp->getObservationDynamics();

        std::shared_ptr<GymInterface> gym;
        if (formalism == "MDP")
            gym = std::make_shared<MDP>(state_space, action_space, reward_space, state_dynamics, start_distribution, horizon, 1.);
        else if (formalism == "MMDP")
            gym = std::make_shared<MMDP>(state_space, action_space, reward_space, state_dynamics, start_distribution, horizon, 1.);
        else if (formalism == "POMDP")
            gym = std::make_shared<POMDP>(state_space, action_space, observation_space, reward_space, state_dynamics, observation_dynamics, start_distribution, horizon, 1.);
        else if (formalism == "MPOMDP")
            gym = std::make_shared<MPOMDP>(state_space, action_space, observation_space, reward_space, state_dynamics, observation_dynamics, start_distribution, horizon, 1.);
        else if (formalism == "BeliefMDP")
            gym = std::make_shared<BeliefMDP>(dpomdp, batch_size);
        else if (formalism == "OccupancyMDP")
            gym = std::make_shared<OccupancyMDP>(dpomdp, memory, true, true, true, true, batch_size);
        else if ((formalism == "PrivateHierarchicalOccupancyMDP") && (qvalue == "tabular"))
            gym = std::make_shared<PrivateHierarchicalOccupancyMDP>(dpomdp, memory, true, true, true, true, batch_size);
        else if ((formalism == "PrivateHierarchicalOccupancyMDP") && ((qvalue == "hierarchical" || (qvalue == "simple-hierarchical"))))
            gym = std::make_shared<PrivateHierarchicalOccupancyMDPWithHistory>(dpomdp, memory, true, true, true, true, batch_size);


    }
    catch (std::exception &e)
    {
        std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
        return sdm::ERROR_UNHANDLED_EXCEPTION;
    }

    return sdm::SUCCESS;

} // END main
