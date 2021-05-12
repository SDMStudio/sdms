#include <iostream>

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/action/stochastic_decision_rule.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

using namespace sdm;

int main(int, char **)
{

    // Joint Deterministic Decision Rule usage
    std::cout << "\n----- Usage : JointDeterministicDecisionRule ( sdm/core/action/joint_det_decision_rule.hpp ) ---------" << std::endl;

    // Here, we will suppose an example with 3 agents
    JointDeterministicDecisionRule<number, number> joint_det_dr;

    // Create the joint deterministic decision rule
    joint_det_dr.setProbability({3, 4, 5}, {2, 1, 3});
    joint_det_dr.setProbability({1, 1, 1}, {3, 2, 3});
    joint_det_dr.setProbability({3, 3, 5}, {0, 5, 7});

    // Displays
    std::cout << "\n#> Joint Decision Rule\n"
              << joint_det_dr << std::endl;

    std::cout << "#> With a_i=2, o_i=5, u_i=7 --> a_i(u_i | o_i) = " << joint_det_dr.getProbability(2, 5, 7) << std::endl;

    std::cout << "#> With o={3, 1, 5}, u={0, 2, 7} --> a(u | o) = " << joint_det_dr.getProbability({3, 1, 5}, {0, 2, 7}) << std::endl;

    std::cout << "#> With o={1, 4, 5}, u={2, 1, 3} --> a(u | o) = " << joint_det_dr.getProbability({1, 4, 5}, {2, 1, 3}) << std::endl;

    // Deterministic Decision Rule usage
    std::cout << "\n----- Usage : DeterministicDecisionRule ( sdm/core/action/det_decision_rule.hpp ) ---------" << std::endl;

    // Create the deterministic decision rule
    DetDecisionRule<number, number> det_dr({0, 1, 2}, {3, 0, 2});

    det_dr.setProbability(3, 4);
    det_dr.setProbability(4, 4, 1.0);
    det_dr.setProbability(5, 4, 0.0);

    // Displays
    std::cout << "\n#> Det Decision Rule\n"
              << det_dr << std::endl;

    std::cout << "#> With o=5, u=4 --> a(u | o) = " << det_dr.getProbability(5, 4) << std::endl;

    // Stochastic Deterministic Decision Rule usage
    std::cout << "\n----- Usage : StochasticDecisionRule ( sdm/core/action/stochastic_decision_rule.hpp ) ---------" << std::endl;

    // Create the stochastic decision rule
    StochasticDecisionRule<number, number> stoch_dr;

    stoch_dr.setProbability(3, 2, 0.5);
    stoch_dr.setProbability(3, 1, 0.5);
    stoch_dr.setProbability(5, 4, 0.8);
    stoch_dr.setProbability(5, 2, 0.2);

    // Displays
    std::cout << "\n#> Stochastic Decision Rule\n"
              << stoch_dr << std::endl;

    std::cout << "#> With o=5, u=4 --> a(u | o) = " << stoch_dr.getProbability(5, 4) << std::endl;
    std::cout << "#> With o=5 --> a(u | o) = " << stoch_dr.getProbabilities(5) << std::endl;

    return 0;
} // END main