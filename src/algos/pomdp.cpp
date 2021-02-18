#include <sdm/algorithms/deep/pomdp/dql.hpp>

using namespace sdm;

namespace po = boost::program_options;

int main(int argv, char** args){
	try {
    po::options_description desc("Allowed options");

		std::string filename, output_file_name, ib_net_filename;
		int episodes;
    number horizon, replay_memory_size, batch_size, i_batch_size, dim_o2, dim_o1, target_update, dim_i, print_every, seed;
    float eps_end, eps_start, eps_decay, discount_factor, epsilon_optimal, rolling_factor, lr, adam_eps;
		bool gpu;

		desc.add_options()
    ("help", "produce help message")
    ("filename,f", po::value<std::string>(&filename)->default_value("tiger"), "set the benchmark filename -- e.g. tiger, mabc, recycling, etc")
    ("output_file_name,o", po::value<std::string>(&output_file_name)->default_value("../runs/log.csv"), "the output .csv file")
		("ib-net-filename,g", po::value<std::string>(&ib_net_filename)->default_value("tiger"), "set the name")
		("horizon,h", po::value<number>(&horizon)->default_value(0), "set the horizon")
    ("episodes,e", po::value<int>(&episodes)->default_value(10), "set the number of episodes")
    ("replay-memory,r", po::value<number>(&replay_memory_size)->default_value(10000), "set the replay memory")
		("optimal-epsilon", po::value<float>(&epsilon_optimal)->default_value(0.0001), "set the epsilon optimal parameter")
    ("eps-end", po::value<float>(&eps_end)->default_value(0), "set the epsilon exploration end value")
    ("eps-start", po::value<float>(&eps_start)->default_value(1), "set the epsilon exploration start value")
    ("eps-decay,j", po::value<float>(&eps_decay)->default_value(1000), "set the epsilon exploration decay speed")
    ("batch-size,b", po::value<number>(&batch_size)->default_value(128), "set the batch size")
		("i-batch-size,l", po::value<number>(&i_batch_size)->default_value(1), "set the i batch size")
    ("o2", po::value<number>(&dim_o2)->default_value(128), "set the history size of agent 0")
    ("o1", po::value<number>(&dim_o1)->default_value(128), "set the history size of agent 1")
    ("target-update,t", po::value<number>(&target_update)->default_value(1000), "set the target update")
		("inner-dim,i", po::value<number>(&dim_i)->default_value(8), "set the inner layer dimensions")
    ("discount-factor,d", po::value<float>(&discount_factor)->default_value(0.99), "set the discount factor")
		("rf", po::value<float>(&rolling_factor)->default_value(0.99), "set the rolling factor")
		("seed", po::value<number>(&seed)->default_value(1), "seed")
    ("lr", po::value<float>(&lr)->default_value(1e-3), "set the learning rate")
    ("adam-eps,a", po::value<float>(&adam_eps)->default_value(1e-8), "set the adam eps")
		("print", po::value<number>(&print_every)->default_value(1), "every how many episodes do we print?")
		("gpu", "if gpu is to be used")
    ;

    po::variables_map vm;

		try{
      po::store(po::parse_command_line(argv, args, desc), vm);

      if(vm.count("help")){
        std::cout << "Basic Command Line Parameter" << std::endl << desc << std::endl;
        return SUCCESS;
      }

      po::notify(vm);
    } catch(po::error& e){
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
      std::cerr << desc << std::endl;
      return ERROR_IN_COMMAND_LINE;
    }

		if(vm.count("gpu")){
      gpu = true;
    } else {
			gpu = false;
		}

		torch::Device device(torch::kCPU);

		if (gpu && torch::cuda::is_available()) {
			device = torch::Device(torch::kCUDA);
		}

		torch::manual_seed(seed);


		std::string bench_filename;
		bench_filename.append("../data/world/dpomdp");
		bench_filename.append("/");
		bench_filename.append(filename);
		bench_filename.append(".dpomdp");

		std::shared_ptr<POSG> game = std::make_shared<sdm::DecPOMDP>(bench_filename);

		game->setDiscount(discount_factor);

		if (horizon == 0 && discount_factor < 1.0){
      horizon = (number)(log((1 - game->getDiscount()) * epsilon_optimal / game->getRewards()[0].getMaxReward()) / log(game->getDiscount()));
    }else if (horizon == 0 && discount_factor == 1.0){
      horizon = 1000;
    }

		game->setPlanningHorizon(horizon);

		std::cout << "episodes " << episodes << std::endl;
		std::cout << "horizon " << horizon << std::endl;
		std::cout << "batch_size " << batch_size << std::endl;
		std::cout << "i_batch_size " << i_batch_size << std::endl;
		std::cout << "dim_o2 " << dim_o2 << std::endl;
		std::cout << "dim_o1 " << dim_o1 << std::endl;
		std::cout << "target_update " << target_update << std::endl;
		std::cout << "dim_i " << dim_i << std::endl;
		std::cout << "print_every " << print_every << std::endl;
		std::cout << "eps_end " << eps_end << std::endl;
		std::cout << "eps_start " << eps_start << std::endl;
		std::cout << "eps_decay " << eps_decay << std::endl;
		std::cout << "discount_factor " << discount_factor << std::endl;
		std::cout << "rolling_factor " << rolling_factor << std::endl;
		std::cout << "lr " << lr << std::endl;
		std::cout << "adam_eps " << adam_eps << std::endl;
		std::cout << "device " << device << std::endl;
		std::cout << "game " << game << std::endl;
		std::cout << "replay_memory_size " << replay_memory_size << std::endl;
		std::cout << "output_file_name " << output_file_name << std::endl;
		std::cout << "ib_net_filename " << ib_net_filename << std::endl;

		DQL dql(
			episodes, 
			horizon, batch_size, i_batch_size, dim_o2, dim_o1, target_update, dim_i, print_every, 
			eps_end, eps_start, eps_decay, discount_factor, rolling_factor, lr, adam_eps, 
			device, game, replay_memory_size, output_file_name, ib_net_filename
		);

		// dql.solve();

	} catch(std::exception& e){
    std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
    return ERROR_UNHANDLED_EXCEPTION;
  }
	
}