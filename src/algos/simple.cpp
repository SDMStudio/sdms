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