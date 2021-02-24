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

#include <sdm/types.hpp>

namespace sdm{
	// Deep Q Network with 3 layers.
	struct Q_NetworkImpl : torch::nn::Module {
		torch::nn::Linear fc;
		Q_NetworkImpl(number, number);
		// Receives the history hi of agent i and produces q values for that agent.
		torch::Tensor forward(torch::Tensor);
	};
	TORCH_MODULE(Q_Network);

	// Recurrent Neural Network.
	struct Transition_NetworkImpl : torch::nn::Module {
		torch::nn::GRUCell gru;
		Transition_NetworkImpl(number, number);
		// Receives history hi, action zi, and observation oi for agent i and return history next_hi.
		torch::Tensor forward(torch::Tensor, torch::Tensor);
	};
	TORCH_MODULE(Transition_Network);

	// Q_Network with 2 RNNs packaged together.
	struct DRQNImpl : torch::nn::Module {
		Transition_Network trans_net_2, trans_net_1;
		Q_Network q_net;
		//
		DRQNImpl(number, number, number, number, number, number);
	};
	TORCH_MODULE(DRQN);

}