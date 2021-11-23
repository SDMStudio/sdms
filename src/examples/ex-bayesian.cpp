#include <memory>
#include <sdm/world/two_players_bayesian_game.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/algorithms/bayesian_game_solver.hpp>
#include <sdm/world/bayesian_game_interface.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string bayesian_file = (argc > 1) ? argv[1] :  "/home/sdms/data/world/bayesian_game/ex1.byg";
    // Instanciate a two players bayesian game from a ".byg" or ".nfg" file
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Instanciate a two players bayesian game" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::shared_ptr<sdm::BayesianGameInterface> bayesian_game = parser::parse_file_bayesian(bayesian_file);

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Initialize the solver" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::shared_ptr<Algorithm> solver = std::make_shared<TwoPlayersBayesianGameSolver>(bayesian_game, 1);
    solver->initialize();

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Solve the bayesian game" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    solver->solve();    
    //solver->terminate();
}
