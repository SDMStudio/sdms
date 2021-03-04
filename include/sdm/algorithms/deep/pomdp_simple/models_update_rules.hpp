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
#include <sdm/algorithms/deep/pomdp_simple/agents.hpp>
#include <sdm/algorithms/deep/pomdp_simple/replay_memory.hpp>

namespace sdm{
	struct POMDP_ModelsUpdateRules_Simple {
		// Batch size of updates i.e. how many transitions at a time we use to update the parameters of the nets.
		number batch_size;
		// CPU or GPU.
		torch::Device device = torch::Device(torch::kCPU);
		// The game to be solved.
		std::shared_ptr<sdm::POSG> game;
		// Initialize the ModelsUpdateRules object.
		POMDP_ModelsUpdateRules_Simple(number, torch::Device, std::shared_ptr<sdm::POSG>&);
		// Do one step of update to the networks.
		// double update(std::shared_ptr<POMDP_ReplayMemory>&, std::shared_ptr<POMDP_Agents>&);
		double update(std::shared_ptr<POMDP_ReplayMemory_Simple>&, std::shared_ptr<POMDP_Agents_Simple>&);
		// Construct the batch which is made up of Tensors of correct dimensions and in the correct device (CPU/GPU).
		pomdp_batch_simple construct_batch(std::vector<pomdp_transition_simple>);
		//
		torch::Tensor get_q_values(torch::Tensor, torch::Tensor, torch::Tensor, DQN&);
		//
		torch::Tensor get_target_q_values(torch::Tensor, torch::Tensor, torch::Tensor, DQN&);
		// Update the parameters of model
		void update_net(std::shared_ptr<POMDP_Agents_Simple>&, torch::Tensor);
	};
}