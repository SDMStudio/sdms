#include <memory>
#include <sdm/world/two_players_bayesian_game.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/algorithms/hs4bg.hpp>
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

    std::shared_ptr<Algorithm> solver = std::make_shared<HS4BG>(bayesian_game);
    solver->initialize();

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Solve the bayesian game" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    solver->solve();

    std::static_pointer_cast<HS4BG>(solver)->saveSolution();

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Decision rule :" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::vector<std::shared_ptr<StochasticDecisionRule>> solution = std::static_pointer_cast<HS4BG> (solver)->getSolution();
    std::cout << solution[0]->str() << std::endl;
    std::cout << solution[1]->str() << std::endl;
}
