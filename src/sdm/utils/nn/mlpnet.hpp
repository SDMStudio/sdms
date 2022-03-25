#pragma once

#include <torch/torch.h>
#include <sdm/types.hpp>

namespace sdm
{
    /**
     * @brief Namespace grouping all neural networks definitions.
     * 
     */
    namespace nn
    {
        struct MlpNetImpl : torch::nn::Module
        {
            MlpNetImpl(sdm::number num_states, sdm::number num_actions)
                : fc1(torch::nn::Linear(num_states, 10)),
                  fc2(torch::nn::Linear(10, num_actions))
            {
                register_module("fc1", fc1);
                register_module("fc2", fc2);
            }
            torch::Tensor forward(torch::Tensor x)
            {
                torch::Tensor state_action_values = fc2(torch::relu(fc1(x)));
                return state_action_values;
            }
            torch::nn::Linear fc1, fc2;
        };
        TORCH_MODULE(MlpNet);
    }
}
