#include <random>
#include <math.h>
#include <vector>
#include <iterator>
#include <tuple>
#include <utility>

#include <boost/program_options.hpp>

#include <sdm/worlds.hpp>

using namespace sdm;

namespace po = boost::program_options;

int main(int argv, char** args){
	try {
    po::options_description desc("Allowed options");

		std::string filename;
    number horizon;
    float discount_factor, epsilon_optimal;


		desc.add_options()
    ("help", "produce help message")
    ("filename,f", po::value<std::string>(&filename)->default_value("tiger"), "set the benchmark filename -- e.g. tiger, mabc, recycling, etc")
		("horizon,h", po::value<number>(&horizon)->default_value(0), "set the horizon")
		("optimal-epsilon", po::value<float>(&epsilon_optimal)->default_value(0.0001), "set the epsilon optimal parameter")
    ("discount-factor,d", po::value<float>(&discount_factor)->default_value(0.99), "set the discount factor")
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

		std::cout << "game->getPlanningHorizon() " << game->getPlanningHorizon() << std::endl;
		std::cout << "game->getNumActions(0) " << game->getNumActions(0) << std::endl;
		std::cout << "game->getNumActions(1) " << game->getNumActions(1) << std::endl;
		std::cout << "game->getNumObservations(0) " << game->getNumObservations(0) << std::endl;
		std::cout << "game->getNumObservations(1) " << game->getNumObservations(1) << std::endl;
		std::cout << "game->getNumAgents() " << game->getNumAgents() << std::endl;
		

	} catch(std::exception& e){
    std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
    return ERROR_UNHANDLED_EXCEPTION;
  }
	
}