#include <memory>
#include <sdm/world/two_players_bayesian_game.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/algorithms/bayesian_game_solver.hpp>
#include <sdm/world/bayesian_game_interface.hpp>
#include <chrono>
#include <iostream>
#include <fstream>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string bayesian_file = (argc > 1) ? argv[1] :  "/home/sdms/data/world/bayesian_game/ex1.nfg";
    int solveForAgent = (argc > 2) ? std::stoi(argv[2]) : 0;

    std::string perf_filename = "null";
    if (argc > 3) perf_filename = argv[3];

    // Instanciate a two players bayesian game from a ".byg" or ".nfg" file
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Instanciate a two players bayesian game" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::shared_ptr<sdm::BayesianGameInterface> bayesian_game = parser::parse_file_bayesian(bayesian_file);

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Initialize the solver" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::shared_ptr<Algorithm> solver = std::make_shared<TwoPlayersBayesianGameSolver>(bayesian_game, solveForAgent);
    
    auto start = std::chrono::high_resolution_clock::now();

    solver->initialize();

    auto step = std::chrono::high_resolution_clock::now();

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Solve the bayesian game" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    solver->solve();
    
    auto end = std::chrono::high_resolution_clock::now();

    if (perf_filename != "null")
    {
        auto lpTime = std::chrono::duration_cast<std::chrono::microseconds>(step - start);
        auto solveTime = std::chrono::duration_cast<std::chrono::microseconds>(end - step);
        ofstream perf_file;
        perf_file.open(perf_filename,std::ios::out | std::ios::app);
        perf_file << bayesian_file + " " + std::to_string(solveForAgent) + " : ";
        perf_file << lpTime.count();
        perf_file << " | ";
        perf_file << solveTime.count();
        perf_file << "\n";
        perf_file.close();

    }


    // std::cout << "----------------------------------------" << std::endl;
    // std::cout << "#> Decision rule :" << std::endl;
    // std::cout << "----------------------------------------" << std::endl;

    // std::cout << std::static_pointer_cast<TwoPlayersBayesianGameSolver> (solver)->getSolution()->str() << std::endl;
}
