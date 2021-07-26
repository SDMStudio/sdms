#include <iostream>

#include <boost/program_options.hpp>

#include <sdm/types.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/algorithms/q_learning.hpp>
#include <sdm/algorithms/deep_q_learning.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/utils/value_function/hierarchical_qvalue_function.hpp>
#include <sdm/utils/rl/exploration.hpp>
#include <sdm/utils/value_function/backup/tabular_qvalue_backup.hpp>
#include <sdm/utils/value_function/backup/simple_hierarchical_qvalue_backup.hpp>
// #include <sdm/utils/value_function/backup/hierarchical_qvalue_backup.hpp>
#include <sdm/utils/rl/experience_memory.hpp>
#include <sdm/utils/rl/deep_experience_memory.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/private_hierarchical_occupancy_mdp.hpp>
#include <sdm/world/private_hierarchical_occupancy_mdp_with_history.hpp>
#include <sdm/world/private_hierarchical_occupancy_mdp_with_observation.hpp>

#include <sdm/core/state/private_occupancy_state.hpp>

#include <sdm/utils/value_function/backup/dqn_mdp_backup.hpp>
#include <sdm/utils/value_function/backup/dqn_bmdp_backup.hpp>
#include <sdm/utils/value_function/backup/dqn_phomdp_backup.hpp>

#include <sdm/utils/nn/dqn.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;
int main(int argc, char **argv)
{
    try
    {
        std::string path, formalism, name, qvalue, version, q_init;
        unsigned long num_episodes;
        number horizon, memory, sampling_size, batch_size, inner_dim;
        double lr, discount, sf, p_b, p_o, p_c, ball_r, epsilon_optimal, exploration_end, final_epsilon;
        int seed;
        int capacity;
        bool store_actions, store_action_spaces;

        po::options_description options("Options");
        options.add_options()
        ("help", "produce help message")("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()
        ("path,p", po::value<string>(&path)->default_value("tiger"), "the path to the problem to be solved")
        ("formalism,f", po::value<string>(&formalism)->default_value("MDP"), "the formalism to use e.g. MDP, MMDP, POMDP, MPOMDP")
        ("lr,l", po::value<double>(&lr)->default_value(0.01), "the learning rate")
        ("p_b", po::value<double>(&p_b)->default_value(0.0001), "the precision of belief state")
        ("p_o", po::value<double>(&p_o)->default_value(0.0001), "the precision of occupancy state ")
        ("p_c", po::value<double>(&p_c)->default_value(0.01), "the precision of compression ")
        ("ball_r", po::value<double>(&ball_r)->default_value(1.0), "the radius of the balls of s in the hqvf")
        ("optimal-epsilon", po::value<double>(&epsilon_optimal)->default_value(0.0001), "set the epsilon optimal parameter")
        ("exp-end", po::value<double>(&exploration_end)->default_value(0.9), "set the epsilon optimal parameter")
        ("final-eps", po::value<double>(&final_epsilon)->default_value(0.1), "set the epsilon optimal parameter")
        ("discount,d", po::value<double>(&discount)->default_value(1.0), "the discount factor")
        ("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")
        ("memory,m", po::value<number>(&memory)->default_value(-1), "the memory. If 0 then infinite memory.")
        ("batch_size,b", po::value<number>(&batch_size)->default_value(32), "batch size")
        ("sampling_size,z", po::value<number>(&sampling_size)->default_value(0), "sampling size")
        ("inner_dim,i", po::value<number>(&inner_dim)->default_value(0), "inner dimension of the NN")
        ("num_episodes,t", po::value<unsigned long>(&num_episodes)->default_value(100000), "number of episodes")
        ("capacity,c", po::value<int>(&capacity)->default_value(1000), "capacity of the experience memory")
        ("seed,s", po::value<int>(&seed)->default_value(1), "random seed")
        ("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment")
        ("store_actions", "store actions")("store_action_spaces", "store action spaces")
        ;

        po::options_description algo_config("Algorithms configuration");
        algo_config.add_options()
        ("qvalue,q", po::value<string>(&qvalue)->default_value("tabular"), "the representation of the Q-Value")
        ("version,v", po::value<string>(&version)->default_value("1"), "the version of hierarchical qvf")
        ("init", po::value<string>(&q_init)->default_value("ZeroInitializer"), "the Q-Value initialization method");

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
        torch::manual_seed(seed);

        auto dpomdp = sdm::parser::parse_file(path);

        auto start_distribution = dpomdp->getStartDistribution();

        auto state_space = dpomdp->getStateSpace();
        auto action_space = dpomdp->getActionSpace();
        auto observation_space = dpomdp->getObservationSpace(0);
        auto reward_space = dpomdp->getRewardSpace();

        auto state_dynamics = dpomdp->getStateDynamics();
        auto observation_dynamics = dpomdp->getObservationDynamics();

        if (horizon == 0 && discount < 1.0)
        {
            horizon = (number)(log((1 - discount) * epsilon_optimal / reward_space->getMaxReward(0)) / log(discount));
        }
        else if (horizon == 0 && discount == 1.0)
        {
            horizon = 1000;
        }

        dpomdp->setHorizon(horizon);
        dpomdp->setDiscount(discount);

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
            gym = std::make_shared<BeliefMDP>(dpomdp, false, false, sampling_size);
        else if (formalism == "OccupancyMDP")
            gym = std::make_shared<OccupancyMDP>(dpomdp, memory, true, true, true, true, sampling_size);
        else if ((formalism == "PrivateHierarchicalOccupancyMDP") && (qvalue == "tabular"))
            gym = std::make_shared<PrivateHierarchicalOccupancyMDP>(dpomdp, memory, true, true, true, true, sampling_size);
        else if ((formalism == "PrivateHierarchicalOccupancyMDP") && ((qvalue == "hierarchical") || (qvalue == "simple-hierarchical")))
            gym = std::make_shared<PrivateHierarchicalOccupancyMDPWithHistory>(dpomdp, memory, true, true, false, false, sampling_size);
        else if ((formalism == "PrivateHierarchicalOccupancyMDP") && (qvalue == "deep"))
            gym = std::make_shared<PrivateHierarchicalOccupancyMDPWithObservation>(dpomdp, memory, true, false, false, false, sampling_size);

        // Set precision
        Belief::PRECISION = p_b;
        OccupancyState::PRECISION = p_o;
        PrivateOccupancyState::PRECISION_COMPRESSION = p_c;

        // Instanciate the initializer
        std::shared_ptr<ZeroInitializer> initializer = std::make_shared<sdm::ZeroInitializer>();

        std::shared_ptr<QValueFunction> q_value_table;
        if (qvalue == "tabular")
            q_value_table = std::make_shared<TabularQValueFunction>(horizon, lr, initializer);
        else if (qvalue == "simple-hierarchical")
            q_value_table = std::make_shared<HierarchicalQValueFunction>(horizon, lr, initializer, ball_r, true);
        else if (qvalue == "hierarchical")
            q_value_table = std::make_shared<HierarchicalQValueFunction>(horizon, lr, initializer, ball_r, true);

        std::shared_ptr<QValueFunction> target_q_value_table;
        if (qvalue == "tabular")
            target_q_value_table = std::make_shared<TabularQValueFunction>(horizon, lr, initializer);
        else if (qvalue == "simple-hierarchical")
            q_value_table = std::make_shared<HierarchicalQValueFunction>(horizon, lr, initializer, ball_r, true);
        else if (qvalue == "hierarchical")
            q_value_table = std::make_shared<HierarchicalQValueFunction>(horizon, lr, initializer, ball_r, true);

        std::shared_ptr<DQN> policy_net;
        std::shared_ptr<DQN> target_net;
        if (qvalue == "deep")
        {
            number x_dim = std::static_pointer_cast<DiscreteSpace>(state_space)->getNumItems();
            number u1_dim = std::static_pointer_cast<DiscreteSpace>(std::static_pointer_cast<MultiDiscreteSpace>(action_space)->get(0))->getNumItems();
            number u2_dim = std::static_pointer_cast<DiscreteSpace>(std::static_pointer_cast<MultiDiscreteSpace>(action_space)->get(1))->getNumItems();
            number z1_dim = std::static_pointer_cast<DiscreteSpace>(std::static_pointer_cast<MultiDiscreteSpace>(observation_space)->get(0))->getNumItems();
            number z2_dim = std::static_pointer_cast<DiscreteSpace>(std::static_pointer_cast<MultiDiscreteSpace>(observation_space)->get(1))->getNumItems();
            std::cout << "|X| = " << x_dim << std::endl << "|U1| = " << u1_dim << std::endl << "|U2| = " << u2_dim << std::endl << "|Z1| = " << z1_dim << std::endl << "|Z2| = " << z2_dim << std::endl;
            if (formalism == "MDP")
            {
                number input_dim = x_dim + horizon;
                if (inner_dim == 0)
                    inner_dim = x_dim + 1;
                number output_dim = u1_dim * u2_dim;
                policy_net = std::make_shared<DQN>(input_dim, inner_dim, output_dim);
                target_net = std::make_shared<DQN>(input_dim, inner_dim, output_dim);
            }
            else if (formalism == "BeliefMDP")
            {
                number input_dim = x_dim + horizon;
                if (inner_dim == 0)
                    inner_dim = x_dim + 1;
                number output_dim = u1_dim * u2_dim;
                policy_net = std::make_shared<DQN>(input_dim, inner_dim, output_dim);
                target_net = std::make_shared<DQN>(input_dim, inner_dim, output_dim);
            }
            else if (formalism == "PrivateHierarchicalOccupancyMDP")
            {
                number input_dim = x_dim * z1_dim + z1_dim + z2_dim + horizon;
                if (inner_dim == 0)
                    inner_dim = x_dim * z1_dim + z1_dim + z2_dim + 1;
                number output_dim = u1_dim * u2_dim;
                policy_net = std::make_shared<DQN>(input_dim, inner_dim, output_dim);
                target_net = std::make_shared<DQN>(input_dim, inner_dim, output_dim);
            }
            std::cout << *policy_net << std::endl;
        }

        // Instanciate exploration process
        std::shared_ptr<EpsGreedy> exploration = std::make_shared<EpsGreedy>(1.0, final_epsilon, 0.0, exploration_end);

        // Instanciate the memory
        std::shared_ptr<ExperienceMemoryInterface> experience_memory;
        if (qvalue != "deep")
            experience_memory = std::make_shared<ExperienceMemory>(horizon);
        else
            experience_memory = std::make_shared<DeepExperienceMemory>(capacity);

        std::shared_ptr<QValueBackupInterface> backup;
        if (qvalue == "tabular")
            backup = std::make_shared<TabularQValueBackup>(experience_memory, q_value_table, q_value_table, discount, horizon);
        else if (qvalue == "simple-hierarchical")
            backup = std::make_shared<SimpleHierarchicalQValueBackup>(experience_memory, q_value_table, q_value_table, discount, horizon, action_space, gym);
        // else if (qvalue == "hierarchical")
        //     backup = std::make_shared<HierarchicalQValueBackup>(experience_memory, q_value_table, q_value_table, discount, horizon, action_space);
        else if ((qvalue == "deep") && (formalism == "MDP"))
            backup = std::make_shared<DqnMdpBackup>(experience_memory, policy_net, target_net, discount, horizon, batch_size, lr, state_space, action_space);
        else if ((qvalue == "deep") && (formalism == "BeliefMDP"))
            backup = std::make_shared<DqnBmdpBackup>(experience_memory, policy_net, target_net, discount, horizon, batch_size, lr, state_space, action_space);
        else if ((qvalue == "deep") && (formalism == "PrivateHierarchicalOccupancyMDP"))
            backup = std::make_shared<DqnPhomdpBackup>(experience_memory, policy_net, target_net, discount, horizon, batch_size, lr, state_space, observation_space, action_space);

        std::shared_ptr<Algorithm> algorithm;
        if (qvalue != "deep")
            algorithm = std::make_shared<QLearning>(gym, experience_memory, q_value_table, q_value_table, backup, exploration, horizon, discount, lr, 1, num_episodes, name);
        else
            algorithm = std::make_shared<DeepQLearning>(gym, experience_memory, policy_net, target_net, backup, exploration, horizon, discount, lr, num_episodes, name);

        algorithm->do_initialize();

        algorithm->do_solve();
    }
    catch (std::exception &e)
    {
        std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
        return sdm::ERROR_UNHANDLED_EXCEPTION;
    }

    return sdm::SUCCESS;

} // END main
