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
#include <sdm/utils/value_function/hierarchical_qvalue_function_v1.hpp>
#include <sdm/utils/value_function/hierarchical_qvalue_function_v2.hpp>
#include <sdm/utils/rl/exploration.hpp>
#include <sdm/utils/value_function/backup/tabular_qvalue_backup.hpp>
#include <sdm/utils/value_function/backup/hierarchical_qvalue_backup_v1.hpp>
#include <sdm/utils/value_function/backup/hierarchical_qvalue_backup_v2.hpp>
#include <sdm/utils/rl/experience_memory.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/private_hierarchical_occupancy_mdp.hpp>

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
        ("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment");

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
            gym = std::make_shared<OccupancyMDP>(dpomdp, memory, true, true, true, batch_size);
        else if (formalism == "PrivateHierarchicalOccupancyMDP")
            gym = std::make_shared<PrivateHierarchicalOccupancyMDP>(dpomdp, memory, true, true, true, batch_size);

        // Set precision
        Belief::PRECISION = p_b;
        OccupancyState::PRECISION = p_o;
        PrivateOccupancyState::PRECISION_COMPRESSION = p_c;

        // std::cout << "MEMORY=" << memory << std::endl;
        // std::cout << "Belief::PRECISION=" << Belief::PRECISION << std::endl;
        // std::cout << "OccupancyState::PRECISION=" << OccupancyState::PRECISION << std::endl;
        // std::cout << "PrivateOccupancyState::PRECISION_COMPRESSION=" << PrivateOccupancyState::PRECISION_COMPRESSION << std::endl;
        // Instanciate the initializer
        std::shared_ptr<ZeroInitializer> initializer = std::make_shared<sdm::ZeroInitializer>();

        std::shared_ptr<QValueFunction> q_value_table;
        if (qvalue == "tabular")
            q_value_table = std::make_shared<TabularQValueFunction>(horizon, lr, initializer);
        else if ((qvalue == "hierarchical") && (version == "1"))
            q_value_table = std::make_shared<HierarchicalQValueFunctionV1>(horizon, lr, initializer);
        else if ((qvalue == "hierarchical") && (version == "2"))
            q_value_table = std::make_shared<HierarchicalQValueFunctionV2>(horizon, lr, initializer, ball_r);

        std::shared_ptr<QValueFunction> target_q_value_table;
        if (qvalue == "tabular")
            target_q_value_table = std::make_shared<TabularQValueFunction>(horizon, lr, initializer);
        else if ((qvalue == "hierarchical") && (version == "1"))
            target_q_value_table = std::make_shared<HierarchicalQValueFunctionV1>(horizon, lr, initializer);
        else if ((qvalue == "hierarchical") && (version == "2"))
            q_value_table = std::make_shared<HierarchicalQValueFunctionV2>(horizon, lr, initializer, ball_r);

        // Instanciate exploration process
        std::shared_ptr<EpsGreedy> exploration = std::make_shared<EpsGreedy>();
        // Instanciate the memory
        std::shared_ptr<ExperienceMemory> experience_memory = std::make_shared<ExperienceMemory>(horizon);

        std::shared_ptr<QValueBackupInterface> backup;
        if (qvalue == "tabular")
            backup = std::make_shared<TabularQValueBackup>(experience_memory, q_value_table, q_value_table, discount);
        else if ((qvalue == "hierarchical") && (version == "1"))
            backup = std::make_shared<HierarchicalQValueBackupV1>(experience_memory, q_value_table, q_value_table, discount, action_space);
        else if ((qvalue == "hierarchical") && (version == "2"))
            backup = std::make_shared<HierarchicalQValueBackupV2>(experience_memory, q_value_table, q_value_table, discount, action_space);

        auto algorithm = std::make_shared<QLearning>(gym, experience_memory, q_value_table, q_value_table, backup, exploration, horizon, discount, lr, 1, max_steps, name);

        algorithm->do_initialize();
        std::cout << "algorithm->do_initialize();" << std::endl;
        algorithm->do_solve();

        algorithm->saveResults(name + "_test_rl.csv", OccupancyState::PRECISION);


        std::cout << "PASSAGE IN NEXT STATE : " << OccupancyMDP::PASSAGE_IN_NEXT_STATE << std::endl;
        std::cout << "MEAN SIZE STATE : " << OccupancyMDP::MEAN_SIZE_STATE << std::endl;
        std::cout << "\nTOTAL TIME IN STEP : " << OccupancyMDP::TIME_IN_STEP << std::endl;
        std::cout << "TOTAL TIME IN APPLY DR : " << OccupancyMDP::TIME_IN_APPLY_DR << std::endl;
        std::cout << "TOTAL TIME IN UNDERLYING STEP : " << OccupancyMDP::TIME_IN_UNDER_STEP << std::endl;
        std::cout << "TOTAL TIME IN GET REWARD : " << OccupancyMDP::TIME_IN_GET_REWARD << std::endl;
        std::cout << "TOTAL TIME IN GET ACTION : " << OccupancyMDP::TIME_IN_GET_ACTION << std::endl;
        std::cout << "TOTAL TIME IN NEXT Occupancy STATE : " << OccupancyMDP::TIME_IN_NEXT_OSTATE << std::endl;
        std::cout << "\nTOTAL TIME IN NEXT STATE : " << OccupancyMDP::TIME_IN_NEXT_STATE << std::endl;
        std::cout << "TOTAL TIME IN COMPRESS : " << OccupancyMDP::TIME_IN_COMPRESS << std::endl;
        std::cout << "\nTOTAL TIME IN Occupancy::operator== : " << OccupancyState::TIME_IN_EQUAL_OPERATOR << std::endl;
        std::cout << "TOTAL TIME IN Occupancy::getProba : " << OccupancyState::TIME_IN_GET_PROBA << std::endl;
        std::cout << "TOTAL TIME IN Occupancy::setProba : " << OccupancyState::TIME_IN_SET_PROBA << std::endl;
        std::cout << "TOTAL TIME IN Occupancy::addProba : " << OccupancyState::TIME_IN_ADD_PROBA << std::endl;
        std::cout << "TOTAL TIME IN Occupancy::finalize : " << OccupancyState::TIME_IN_FINALIZE << std::endl;
        std::cout << "OccupancyState::TIME_IN_HASH : " << OccupancyState::TIME_IN_HASH << std::endl;
    }
    catch (std::exception &e)
    {
        std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
        return sdm::ERROR_UNHANDLED_EXCEPTION;
    }

    return sdm::SUCCESS;

} // END main
