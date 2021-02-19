/**
 * @file ex5.cpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that give some usage exemples for pytorch
 * @version 1.0
 * @date 01/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <torch/torch.h>
#include <iostream>

int main(){
	std::stringstream stream;

	std::cout << "tensor1:" << std::endl;
	torch::Tensor tensor1 = torch::eye(3);
	torch::save(tensor1, stream);
	std::cout << tensor1 << std::endl;

	std::cout << "tensor2:" << std::endl;
	torch::Tensor tensor2;
	torch::load(tensor2, stream);
	std::cout << tensor2 << std::endl;
}