#include <sdm/algorithms/deep/pomdp_simple/dql.hpp>

using namespace sdm;

namespace po = boost::program_options;

int main(int argv, char** args){
	try {
    po::options_description desc("Allowed options");

		std::string filename, ib_net_filename;
		int episodes;
    number horizon, replay_memory_size, batch_size, n, dim_i, target_update, print_every, seed;
    float eps_end, eps_start, eps_decay, discount_factor, epsilon_optimal, rolling_factor, lr, adam_eps;
		bool gpu;

		desc.add_options()
    ("help", "produce help message")
    ("filename,f", po::value<std::string>(&filename)->default_value("tiger"), "set the benchmark filename -- e.g. tiger, mabc, recycling, etc")
		("ib-net-filename,g", po::value<std::string>(&ib_net_filename)->default_value("tiger"), "set the name")
		("horizon,h", po::value<number>(&horizon)->default_value(0), "set the horizon")
    ("episodes,e", po::value<int>(&episodes)->default_value(10), "set the number of episodes")
    ("replay-memory,r", po::value<number>(&replay_memory_size)->default_value(10000), "set the replay memory")
		("optimal-epsilon", po::value<float>(&epsilon_optimal)->default_value(0.0001), "set the epsilon optimal parameter")
    ("eps-end", po::value<float>(&eps_end)->default_value(0), "set the epsilon exploration end value")
    ("eps-start", po::value<float>(&eps_start)->default_value(1), "set the epsilon exploration start value")
    ("eps-decay,j", po::value<float>(&eps_decay)->default_value(1000), "set the epsilon exploration decay speed")
    ("batch-size,b", po::value<number>(&batch_size)->default_value(1024), "set the batch size")
    ("n,n", po::value<number>(&n)->default_value(128), "number of previous turns of history to keep")
    ("i,i", po::value<number>(&dim_i)->default_value(256), "set the inner dimension")
    ("target-update,t", po::value<number>(&target_update)->default_value(1000), "set the target update")
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

		DQL_Simple dql(
			episodes,
			horizon, batch_size, n, dim_i, target_update, print_every, seed,
			eps_end, eps_start, eps_decay, discount_factor, rolling_factor, lr, adam_eps, 
			device, game, replay_memory_size, ib_net_filename
		);

		dql.solve();


	} catch(std::exception& e){
    std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
    return ERROR_UNHANDLED_EXCEPTION;
  }
	
}