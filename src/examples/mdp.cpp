#include <torch/torch.h>
#include <random>
#include <math.h>
#include <vector>
#include <iterator>
#include <experimental/algorithm>
#include <tuple>
#include <utility>

#include <boost/program_options.hpp>
#include <boost/any.hpp>

#include <sdm/types.hpp>


using namespace sdm;

namespace po = boost::program_options;

typedef float reward;

typedef std::tuple<state, action, reward, state> transition;

std::default_random_engine GENERATOR;

std::uniform_real_distribution<double> DISTRIBUTION_EPS(0.0, 1.0);

int STEPS_DONE = 0;

// Create the device we pass around based on whether CUDA is available.
torch::Device device(torch::kCPU);


struct ReplayMemory {

	int capacity;
	std::vector<transition> memory;
	int position;

	ReplayMemory(int capacity) {
		this->capacity = capacity;
		this->position = 0;
	}

	void push(transition t){
		if (this->memory.size() < this->capacity) {
			this->memory.push_back(std::make_tuple(0, 0, 0, 0));
		}
		this->memory[this->position] = t;
		this->position = (this->position + 1) % this->capacity;
	}

	std::vector<transition> sample(po::variables_map vm){
		std::vector<transition> out;
		std::experimental::sample(this->memory.begin(), this->memory.end(), std::back_inserter(out), boost::any_cast<number>(vm.at("batch-size").value()), std::mt19937{std::random_device{}()});
		return out;
	}

	int size(){
		return this->memory.size();
	}
};

struct DQN2Impl : torch::nn::Module {
  DQN2Impl()
      : fc1(torch::nn::Linear(2, 10)),
				fc2(torch::nn::Linear(10, 9))
	{
		register_module("fc1", fc1);
		register_module("fc2", fc2);
	}
	torch::Tensor forward(torch::Tensor x) {
		torch::Tensor state_action_values = fc2(torch::relu(fc1(x)));
		return state_action_values;
	}
 	torch::nn::Linear fc1, fc2;
};
TORCH_MODULE(DQN2);

namespace
{
  const size_t SUCCESS = 0;
  const size_t ERROR_IN_COMMAND_LINE = 1;
  const size_t ERROR_UNHANDLED_EXCEPTION = 2;
} // namespace

int main(int argv, char** args){
	try {
    po::options_description desc("Allowed options");

    number planning_horizon, replay_memory, episodes, batch_size, hidden_state_size, target_update;
    float eps_end, eps_start, eps_decay, discount_factor, epsilon_optimal;

		desc.add_options()
    ("help", "produce help message")
		("planning_horizon,h", po::value<number>(&planning_horizon)->default_value(0), "set the planning horizon")
    ("episodes,e", po::value<number>(&episodes)->default_value(10), "set the number of episodes per trial")
    ("replay-memory-limit,r", po::value<number>(&replay_memory)->default_value(10000), "set the replay memory  limit")
		("optimal-epsilon,o", po::value<float>(&epsilon_optimal)->default_value(0.0001), "set the epsilon optimal parameter")
    ("epsilon-end,i", po::value<float>(&eps_end)->default_value(0.0001), "set the epsilon exploration end value")
    ("epsilon-start,j", po::value<float>(&eps_start)->default_value(1), "set the epsilon exploration start value")
    ("epsilon-decay,k", po::value<float>(&eps_decay)->default_value(1000), "set the epsilon exploration decay speed")
    ("batch-size,b", po::value<number>(&batch_size)->default_value(128), "set the batch_size")
    ("hidden-state-size,s", po::value<number>(&hidden_state_size)->default_value(128), "set the hidden_state_size")
    ("target-update,t", po::value<number>(&target_update)->default_value(1000), "set the target_update")
    ("discount-factor,d", po::value<float>(&discount_factor)->default_value(0.99), "set the discount factor")
    ;


    po::variables_map vm;
		try{
      po::store(po::parse_command_line(argv, args, desc), vm);

      if(vm.count("help")){
        std::cout << "Basic Command Line Parameter" << std::endl << desc << std::endl;
        return SUCCESS;
      }

      po::notify(vm);
    } catch(po::error& e){
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
      std::cerr << desc << std::endl;
      return ERROR_IN_COMMAND_LINE;
    }


		torch::manual_seed(1);

		if (torch::cuda::is_available()) {
			std::cout << "CUDA is available! Training on GPU." << std::endl;
			device = torch::Device(torch::kCUDA);
		}
		
		DQN2 policy_net;
		DQN2 target_net;
		
		torch::optim::Adam optimizer(policy_net->parameters());

		policy_net->to(device);
		target_net->to(device);

		std::stringstream stream;
		torch::save(policy_net, stream);
		torch::load(target_net, stream);

		ReplayMemory memory(replay_memory);

	} catch(std::exception& e){
    std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
    return ERROR_UNHANDLED_EXCEPTION;
  }
	
}