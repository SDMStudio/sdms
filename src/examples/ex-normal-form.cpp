#include <memory>
#include <sdm/world/two_players_normal_form_game.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/algorithms/bayesian_game_solver.hpp>
#include <sdm/world/bayesian_game_interface.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string nf_file = (argc > 1) ? argv[1] :  "/home/sdms/data/world/bayesian_game/ex1.nfg";
    // Instanciate a two players bayesian game from a ".nfg" file
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Instanciate a two players normal form game" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::shared_ptr<sdm::BayesianGameInterface> nfGame = parser::parse_file_normal_form(nf_file);

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Initialize the solver" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::shared_ptr<Algorithm> solver = std::make_shared<TwoPlayersBayesianGameSolver>(nfGame, 1);
    solver->initialize();

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Solve the bayesian game" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    solver->solve();    
    //solver->terminate();
}
