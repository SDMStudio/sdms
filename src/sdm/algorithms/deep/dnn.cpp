#include <sdm/algorithms/deep/dnn.hpp>

namespace sdm{
	DQNImpl::DQNImpl(number input_dim, number inner_dim, number output_dim)
		: fc1(torch::nn::Linear(input_dim, inner_dim)),
			fc2(torch::nn::Linear(inner_dim, inner_dim)),
			fc3(torch::nn::Linear(inner_dim, output_dim))
	{
		register_module("fc1", fc1);
		register_module("fc2", fc2);
		register_module("fc3", fc3);
	}
	torch::Tensor DQNImpl::forward(torch::Tensor s){
		// Pass the input Tensor to the first fully connected (fc) layer, then apply ReLU. Do the same for the 2nd fc layer. Finally pass it through 3rd fc layer.
		torch::Tensor state_action_values = fc3(torch::relu(fc2(torch::relu(fc1(s)))));
		// We got the state action (q) values. Return them.
		return state_action_values;
	}

	RNNImpl::RNNImpl(number input_dim, number hidden_dim)
		: r1(torch::nn::RNNCell(input_dim, hidden_dim))
	{	
		register_module("r1", r1);
	}
	torch::Tensor RNNImpl::forward(torch::Tensor inputs, torch::Tensor s) {
		// Using the input Tensor and hidden state Tensor, produce the next hidden state and return it.
		return r1(inputs, s);
	}


  DRQNImpl::DRQNImpl(
		number rnn_2_input_size, number rnn_2_hidden_state_size, 
		number rnn_1_input_size, number rnn_1_hidden_state_size, 
		number dqn_input_size, number dqn_inner_dim, number dqn_output_size
	)
		: rnn_2(RNN(rnn_2_input_size, rnn_2_hidden_state_size)),
			rnn_1(RNN(rnn_1_input_size, rnn_1_hidden_state_size)),
			dqn(DQN(dqn_input_size, dqn_inner_dim, dqn_output_size))
	{	
		register_module("rnn_2", rnn_2);
		register_module("rnn_1", rnn_1);
		register_module("dqn", dqn);
	}

}