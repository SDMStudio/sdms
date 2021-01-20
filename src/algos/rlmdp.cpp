#include <torch/torch.h>

#include <sdm/worlds.hpp>
#include <sdm/algorithms/dqn_mdp.hpp>
#include <sdm/utils/rl/eps_greedy.hpp>
#include <sdm/utils/rl/replay_memory.hpp>
#include <sdm/utils/nn/mlpnet.hpp>

using namespace sdm;

namespace po = boost::program_options;

// Create the device we pass around based on whether CUDA is available.
torch::Device device(torch::kCPU);


int main(int argv, char **args)
{
    try
    {
        // program options
        po::options_description desc("Allowed options");

        number planning_horizon, replay_memory, episodes, batch_size, hidden_state_size, target_update;
        float eps_end, eps_start, eps_decay, discount_factor, epsilon_optimal;

        desc.add_options()("help", "produce help message")("planning_horizon,h", po::value<number>(&planning_horizon)->default_value(0), "set the planning horizon")("episodes,e", po::value<number>(&episodes)->default_value(10), "set the number of episodes per trial")("replay-memory-limit,r", po::value<number>(&replay_memory)->default_value(10000), "set the replay memory  limit")("optimal-epsilon,o", po::value<float>(&epsilon_optimal)->default_value(0.0001), "set the epsilon optimal parameter")("epsilon-end,i", po::value<float>(&eps_end)->default_value(0.0001), "set the epsilon exploration end value")("epsilon-start,j", po::value<float>(&eps_start)->default_value(1), "set the epsilon exploration start value")("epsilon-decay,k", po::value<float>(&eps_decay)->default_value(1000), "set the epsilon exploration decay speed")("batch-size,b", po::value<number>(&batch_size)->default_value(128), "set the batch_size")("hidden-state-size,s", po::value<number>(&hidden_state_size)->default_value(128), "set the hidden_state_size")("target-update,t", po::value<number>(&target_update)->default_value(1000), "set the target_update")("discount-factor,d", po::value<float>(&discount_factor)->default_value(0.99), "set the discount factor");

        po::variables_map vm;
        try
        {
            po::store(po::parse_command_line(argv, args, desc), vm);

            if (vm.count("help"))
            {
                std::cout << "Basic Command Line Parameter" << std::endl
                          << desc << std::endl;
                return SUCCESS;
            }

            po::notify(vm);
        }
        catch (po::error &e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl
                      << std::endl;
            std::cerr << desc << std::endl;
            return ERROR_IN_COMMAND_LINE;
        }

        // ----------------
        // Torch config
        torch::manual_seed(1);

        if (torch::cuda::is_available())
        {
            std::cout << "CUDA is available! Training on GPU." << std::endl;
            device = torch::Device(torch::kCUDA);
        }

        // --------------------
        // Instantiate tiger world
        std::shared_ptr<POSG> tiger_pb = std::make_shared<sdm::DecPOMDP>("../data/world/dpomdp/tiger.dpomdp");

        tiger_pb->setDiscount(discount_factor);
        if (planning_horizon == 0 && discount_factor < 1.0)
        {
            planning_horizon = (number)(log((1 - tiger_pb->getDiscount()) * epsilon_optimal / tiger_pb->getRewards()[0].getMaxReward()) / log(tiger_pb->getDiscount()));
        }
        else if (planning_horizon == 0 && discount_factor == 1.0)
        {
            planning_horizon = 1000;
        }
        tiger_pb->setPlanningHorizon(planning_horizon);

        // ------------------------
        // Instantiate DQN algo and solve problem using it

        // Instantiate neural networks models
        DQN policy_net(tiger_pb->getNumStates(), tiger_pb->getNumJActions()), target_net(tiger_pb->getNumStates(), tiger_pb->getNumJActions());

        // // Instantiate replay memory and eps greedy
        ReplayMemory memory(replay_memory);
        EpsGreedy eps_greedy(eps_start, eps_end, eps_decay);

        // // Instantiate DQN algo for MDP problems
        DQNMDP dqn_algo(policy_net, target_net, memory, eps_greedy, batch_size, target_update, device);
        dqn_algo.to(device);

        dqn_algo.solve(tiger_pb, episodes);
    }
    catch (std::exception &e)
    {
        std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
        return ERROR_UNHANDLED_EXCEPTION;
    }
}