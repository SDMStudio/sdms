#include <memory>
#include <sdm/world/two_players_bayesian_game.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/algorithms/bayesian_game_solver.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string bayesian_file = (argc > 1) ? argv[1] :  "/home/tdambrin/sdms/data/world/bayesian_game/ex1.byg";
    // Instanciate a two players bayesian game from a ".byg" file
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Instanciate a two players bayesian game" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::shared_ptr<TwoPlayersBayesianGame> bayesian_game = parser::parse_file_bayesian(bayesian_file);

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Initialize the solver" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    TwoPlayersBayesianGameSolver solver;
    solver.initialize();
    solver.getLPFromBayesianGame(*bayesian_game, 0);

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Solve the bayesian game" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    solver.solve();
    solver.terminate();
}
