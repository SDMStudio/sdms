#include <iostream>
#include <sdm/config.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/algorithms/planning/backward_induction.hpp>

using namespace sdm;

int main()
{
    // Parse the problem file
    auto pomdp = sdm::parser::parsePOMDP(sdm::config::PROBLEM_PATH + "dpomdp/tiger.dpomdp");
    pomdp->setHorizon(4);
    // Recast the problem instance into a solvable interface
    auto belief_mdp = std::make_shared<BeliefMDP>(pomdp);
    // Instanciate the algorithm
    auto algo = std::make_shared<BackwardInduction>(belief_mdp);
    // Initialize and solve
    algo->initialize();
    algo->solve();
}