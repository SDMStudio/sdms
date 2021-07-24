#pragma once

#include <torch/torch.h>
#include <sdm/types.hpp>

namespace sdm
{
    class DQNImpl : public torch::nn::Module
    {
        public:
            DQNImpl(number input_dim, number inner_dim, number output_dim) : fc1(torch::nn::Linear(input_dim, inner_dim)), fc2(torch::nn::Linear(inner_dim, output_dim))
            {
                register_module("fc1", fc1);
                register_module("fc2", fc2);
            }
            torch::Tensor forward(torch::Tensor x)
            {
                return this->fc2(torch::relu(this->fc1(x)));
            }
            torch::nn::Linear fc1, fc2;
    };
    TORCH_MODULE(DQN);
} // namespace sdm