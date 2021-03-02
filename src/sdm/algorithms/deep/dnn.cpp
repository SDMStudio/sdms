#include <sdm/algorithms/deep/dnn.hpp>

namespace sdm{

	Q_NetworkImpl::Q_NetworkImpl(number input_dim, number output_dim)
		: fc1(torch::nn::Linear(input_dim, output_dim))
	{
		register_module("fc1", fc1);
	}
	torch::Tensor Q_NetworkImpl::forward(torch::Tensor s){
		// Pass the input Tensor to the fully connected layer and directly obtain the q values.
		torch::Tensor q_values = fc1(s);
		// We got the state action (q) values. Return them.
		return q_values;
	}

	Gated_RNNImpl::Gated_RNNImpl(number input_dim, number hidden_dim)
		: gru(torch::nn::GRUCell(input_dim, hidden_dim))
	{	
		register_module("gru", gru);
	}
	torch::Tensor Gated_RNNImpl::forward(torch::Tensor inputs, torch::Tensor s) {
		// Using the input Tensor and hidden state Tensor, produce the next hidden state and return it.
		return gru(inputs, s);
	}

	Gated_Recurrent_Q_NetworkImpl::Gated_Recurrent_Q_NetworkImpl(
		number trans_net_2_input_size, number trans_net_2_hidden_state_size, 
		number trans_net_1_input_size, number trans_net_1_hidden_state_size, 
		number q_net_input_size, number q_net_inner_dim, number q_net_output_size
	)
		: trans_net_2(Gated_RNN(trans_net_2_input_size, trans_net_2_hidden_state_size)),
			trans_net_1(Gated_RNN(trans_net_1_input_size, trans_net_1_hidden_state_size)),
			q_net(Q_Network(q_net_input_size, q_net_output_size))
	{	
		register_module("trans_net_2", trans_net_2);
		register_module("trans_net_1", trans_net_1);
		register_module("q_net", q_net);
	}

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

  Deep_Recurrent_Q_NetworkImpl::Deep_Recurrent_Q_NetworkImpl(
		number trans_net_2_input_size, number trans_net_2_hidden_state_size, 
		number trans_net_1_input_size, number trans_net_1_hidden_state_size, 
		number q_net_input_size, number q_net_inner_dim, number q_net_output_size
	)
		: trans_net_2(RNN(trans_net_2_input_size, trans_net_2_hidden_state_size)),
			trans_net_1(RNN(trans_net_1_input_size, trans_net_1_hidden_state_size)),
			q_net(DQN(q_net_input_size, q_net_inner_dim, q_net_output_size))
	{	
		register_module("trans_net_2", trans_net_2);
		register_module("trans_net_1", trans_net_1);
		register_module("q_net", q_net);
	}

}