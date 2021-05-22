#include <iostream>
#include <boost/program_options.hpp>

#include <sdm/types.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/world/hierarchical_private_occupancy_mdp.hpp>
#include <sdm/algorithms/q_learning.hpp>
#include <sdm/algorithms/hierarchical_q_learning.hpp>
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/utils/value_function/hierarchical_qvalue_function.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;

int learn(int argv, char **args)
{
    try
    {
        std::string problem, algorithm, formalism, name, qvalue, q_init;
        unsigned long max_steps;
        number horizon;
        double lr, discount, sf;

        po::options_description options("Options");
        options.add_options()
        ("help", "produce help message")
        ("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()
        ("algorithm,a", po::value<string>(&algorithm)->default_value("qlearning"), "the learning algorithm to use")
        ("problem,p", po::value<string>(&problem)->default_value("tiger"), "the problem to be solved")
        ("formalism,f", po::value<string>(&formalism)->default_value("normal"), "the formalism to use")
        ("lr,l", po::value<double>(&lr)->default_value(0.01), "the learning rate")
        ("smooth,s", po::value<double>(&sf)->default_value(0.999), "the smoothing factor for the E[R]")
        ("discount,d", po::value<double>(&discount)->default_value(0.9), "the discount factor")
        ("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")
        ("max_steps,t", po::value<unsigned long>(&max_steps)->default_value(100000), "the maximum number of timesteps")
        ("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment");

        po::options_description algo_config("Algorithms configuration");
        algo_config.add_options()
        ("qvalue,q", po::value<string>(&qvalue)->default_value("tabular"), "the representation of the Q-Value")
        ("init,i", po::value<string>(&q_init)->default_value("ZeroInitializer"), "the Q-Value initialization method");

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

        using TObservation = number;
        using TState = number;

        using TActionDescriptor = number;
        using TStateDescriptor = Joint<HistoryTree_p<TObservation>>;

        using TActionPrescriptor = HierarchicalPrivateJointDeterministicDecisionRule<TStateDescriptor, TActionDescriptor>;
        using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;

        using env_type = HierarchicalPrivateOccupancyMDP<TStatePrescriptor, TActionPrescriptor>;
        auto HierarchicalPrivateOccupancyMDP = std::make_shared<env_type>(problem, horizon);

        HierarchicalPrivateOccupancyMDP->getUnderlyingProblem()->setDiscount(discount);
        HierarchicalPrivateOccupancyMDP->getUnderlyingProblem()->setPlanningHorizon(horizon);
        HierarchicalPrivateOccupancyMDP->getUnderlyingProblem()->setupDynamicsGenerator();

        if (formalism == "exhaustive")
        {
            // Instanciate initializers and qvalue functions
            auto initializer = std::make_shared<sdm::ZeroInitializer<TStatePrescriptor, TActionPrescriptor>>();
            auto qvalue = std::make_shared<sdm::MappedQValueFunction<TStatePrescriptor, TActionPrescriptor>>(horizon, lr, initializer);

            auto initializer_target = std::make_shared<sdm::ZeroInitializer<TStatePrescriptor, TActionPrescriptor>>();
            auto target_qvalue = std::make_shared<sdm::MappedQValueFunction<TStatePrescriptor, TActionPrescriptor>>(horizon, lr, initializer_target);

            // Instanciate exploration process
            auto exploration_process = std::make_shared<sdm::EpsGreedy<TStatePrescriptor, TActionPrescriptor>>();

            auto problem = std::shared_ptr<GymInterface<TStatePrescriptor, TActionPrescriptor>>(HierarchicalPrivateOccupancyMDP);

            auto algo =  std::make_shared<QLearning<TStatePrescriptor, TActionPrescriptor>>(problem, qvalue, target_qvalue, exploration_process, horizon, discount, lr, 1, max_steps, name);

            algo->do_initialize();
            algo->do_solve();
        }
        else if (formalism == "hierarchical")
        {
            // Instanciate initializers and qvalue functions
            auto initializer = std::make_shared<sdm::ZeroInitializer<TStateDescriptor, Joint<TActionPrescriptor::output>>>();
            auto qvalue = std::make_shared<sdm::HierarchicalMappedQValueFunction<Pair<TStatePrescriptor, TStateDescriptor>, Joint<TActionPrescriptor::output>>>(horizon, lr, initializer, HierarchicalPrivateOccupancyMDP);

            auto initializer_target = std::make_shared<sdm::ZeroInitializer<TStateDescriptor, Joint<TActionPrescriptor::output>>>();
            auto target_qvalue = std::make_shared<sdm::HierarchicalMappedQValueFunction<Pair<TStatePrescriptor, TStateDescriptor>, Joint<TActionPrescriptor::output>>>(horizon, lr, initializer_target, HierarchicalPrivateOccupancyMDP);

            // Instanciate exploration process
            auto exploration_process = std::make_shared<sdm::EpsGreedy<TStatePrescriptor, TActionPrescriptor>>();

            auto algo =  std::make_shared<HierarchicalQLearning<TStatePrescriptor, TActionPrescriptor>>(HierarchicalPrivateOccupancyMDP, qvalue, target_qvalue, exploration_process, horizon, discount, lr, sf, 1, max_steps, name);

            algo->do_initialize();
            algo->do_solve();
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