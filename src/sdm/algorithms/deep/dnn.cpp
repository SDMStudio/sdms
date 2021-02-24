#include <sdm/algorithms/deep/dnn.hpp>

namespace sdm{
	Q_NetworkImpl::Q_NetworkImpl(number input_dim, number output_dim)
		: fc(torch::nn::Linear(input_dim, output_dim))
	{
		register_module("fc", fc);
	}
	torch::Tensor Q_NetworkImpl::forward(torch::Tensor s){
		// Pass the input Tensor to the fully connected layer and directly obtain the q values.
		torch::Tensor q_values = fc(s);
		// We got the state action (q) values. Return them.
		return q_values;
	}

	Transition_NetworkImpl::Transition_NetworkImpl(number input_dim, number hidden_dim)
		: gru(torch::nn::GRUCell(input_dim, hidden_dim))
	{	
		register_module("gru", gru);
	}
	torch::Tensor Transition_NetworkImpl::forward(torch::Tensor inputs, torch::Tensor s) {
		// Using the input Tensor and hidden state Tensor, produce the next hidden state and return it.
		return gru(inputs, s);
	}


  DRQNImpl::DRQNImpl(
		number trans_net_2_input_size, number trans_net_2_hidden_state_size, 
		number trans_net_1_input_size, number trans_net_1_hidden_state_size, 
		number q_net_input_size, number q_net_output_size
	)
		: trans_net_2(Transition_Network(trans_net_2_input_size, trans_net_2_hidden_state_size)),
			trans_net_1(Transition_Network(trans_net_1_input_size, trans_net_1_hidden_state_size)),
			q_net(Q_Network(q_net_input_size, q_net_output_size))
	{	
		register_module("trans_net_2", trans_net_2);
		register_module("trans_net_1", trans_net_1);
		register_module("q_net", q_net);
	}

}