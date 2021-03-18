#pragma once

#include <random>
#include <math.h>
#include <vector>
#include <iterator>
#include <experimental/algorithm>
#include <tuple>
#include <utility>

#include <boost/program_options.hpp>
#include <boost/any.hpp>

#include <torch/torch.h>

#include <sdm/worlds.hpp>
#include <sdm/algorithms/deep/osdpomdp/agents.hpp>
#include <sdm/algorithms/deep/osdpomdp/replay_memory.hpp>

namespace sdm{
	struct ModelsUpdateRules {
		// Random engine for the object below.
		std::default_random_engine random_engine;
		// Uniform real distribution of numbers between 0 and 1, to compare with alpha.
		std::uniform_real_distribution<double> uniform_alpha_distribution;
		// If we use induced bias or not i.e. the solution of POMDP.
		bool induced_bias;
		// Batch size of updates i.e. how many transitions at a time we use to update the parameters of the nets.
		number batch_size;
		// Sampling memory size (|S|) i.e. how many sampled histories of agent 1 do we store per transition.
		number sampling_memory_size;
		// CPU or GPU.
		torch::Device device = torch::Device(torch::kCPU);
		// The game to be solved.
		std::shared_ptr<sdm::POSG> game;
		// Initialize the ModelsUpdateRules object.
		ModelsUpdateRules(number, number, torch::Device, std::shared_ptr<sdm::POSG>&, bool);
		// Do one step of update to the networks.
		double update(std::shared_ptr<ReplayMemory>&, std::shared_ptr<Agents>&, float);
		// Construct the batch which is made up of Tensors of correct dimensions and in the correct device (CPU/GPU).
		batch construct_batch(std::vector<transition>);
		// {Q}_{t}^{1} = {Q}^{1}(o_{t}^{0}, o_{t}^{1, (n)}, a_{t}^{0}, a_{t}^{1, (n)}, \{o_{t}^{1, (m))}\}_{m=0}^{M-1})
		torch::Tensor get_q_values(torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, DQN&);
		// {TargetQ}_{t}^{1} = r_{t} + \gamma \cdot \left [(1-\alpha) \cdot \max_{a'}  \mathbf{Q}^{1}(o_{t+1}^{0}, o_{t+1}^{1,(n)}, a_{t+1}^{0}, a', \{o_{t+1}^{1, (m)}\}_{m=0}^{M-1}) + \alpha \cdot (\max_{a'}  {Q}^{InducedBias}(o_{t+1}^{0}, o_{t+1}^{1,(n)}, a')) \right]
		torch::Tensor get_target_q_values(torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, std::shared_ptr<Agents>&, float);
		// Update the parameters of agent 1's policy net.
		void update_policy_net(std::shared_ptr<Agents>&, torch::Tensor);
	};
}