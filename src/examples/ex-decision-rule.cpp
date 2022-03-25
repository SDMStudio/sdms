// #include <sdm/core/actions.hpp>
// #include <sdm/core/state/base_state.hpp>

// using namespace sdm;

// int main(int, char **)
// {
//     auto state_0 = std::make_shared<DiscreteState>(0);
//     auto state_1 = std::make_shared<DiscreteState>(1);
//     auto state_2 = std::make_shared<DiscreteState>(2);
//     auto state_3 = std::make_shared<DiscreteState>(3);
//     auto state_4 = std::make_shared<DiscreteState>(4);
//     auto state_5 = std::make_shared<DiscreteState>(5);

//     auto action_0 = std::make_shared<DiscreteAction>(0);
//     auto action_1 = std::make_shared<DiscreteAction>(1);
//     auto action_2 = std::make_shared<DiscreteAction>(2);
//     auto action_3 = std::make_shared<DiscreteAction>(3);
//     auto action_4 = std::make_shared<DiscreteAction>(4);
//     auto action_5 = std::make_shared<DiscreteAction>(5);

//     // Joint Deterministic Decision Rule usage
//     std::cout << "\n----- Usage : JointDeterministicDecisionRule ( sdm/core/action/joint_det_decision_rule.hpp ) ---------" << std::endl;

//     // Here, we will suppose an example with 3 agents
//     JointDeterministicDecisionRule joint_det_dr;

//     auto joint_state_1 = JointState(std::vector<std::shared_ptr<State>>({state_3,state_4,state_5}));
//     auto joint_state_2 = JointState(std::vector<std::shared_ptr<State>>({state_1,state_1,state_1}));
//     auto joint_state_3 = JointState(std::vector<std::shared_ptr<State>>({state_3,state_1,state_5}));

//     auto joint_action_1 = JointAction(std::vector<std::shared_ptr<Action>>({action_2,action_1,action_3}));
//     auto joint_action_2 = JointAction(std::vector<std::shared_ptr<Action>>({action_3,action_2,action_3}));
//     auto joint_action_3 = JointAction(std::vector<std::shared_ptr<Action>>({action_0,action_5,action_1}));

//     // Create the joint deterministic decision rule
//     joint_det_dr.setProbability(joint_state_1, joint_action_1);
//     joint_det_dr.setProbability(joint_state_2, joint_action_2);
//     joint_det_dr.setProbability(joint_state_3, joint_action_3);

//     // Displays
//     std::cout << "\n#> Joint Decision Rule\n"
//               << joint_det_dr << std::endl;

//     std::cout << "#> With a_i=2, o_i=5, u_i=1 --> a_i(u_i | o_i) = " << joint_det_dr.getProbability(state_5, action_1,2) << std::endl;

//     std::cout << "#> With o={3, 1, 5}, u={0, 5, 1} --> a(u | o) = " << joint_det_dr.getProbability(joint_state_3, joint_action_3) << std::endl;

//     std::cout << "#> With o={3, 1, 5}, u={2, 1, 3} --> a(u | o) = " << joint_det_dr.getProbability(joint_state_3, joint_action_1) << std::endl;

//     // Determinis*tic Decision Rule usage
//     std::cout << "\n----- Usage : DeterministicDecisionRule ( sdm/core/action/det_decision_rule.hpp ) ---------" << std::endl;

//     // Create the deterministic decision rule
//     std::vector<std::shared_ptr<State>> state_vector = {state_0, state_1,state_2};
//     std::vector<std::shared_ptr<Action>> action_vector = {action_3, action_0,action_2};

//     DeterministicDecisionRule det_dr(state_vector,action_vector);

//     det_dr.setProbability(state_3, action_4);
//     det_dr.setProbability(state_4, action_4, 1.0);
//     det_dr.setProbability(state_5, action_4, 0.0);

//     // Displays
//     std::cout << "\n#> Det Decision Rule\n"
//               << det_dr << std::endl;

//     std::cout << "#> With o=4, u=4 --> a(u | o) = " << det_dr.getProbability(state_4, action_4) << std::endl;
//     // std::cout << "#> With o=5, u=4 --> a(u | o) = " << det_dr.getProbability(state_5, action_4) << std::endl;

//     // Stochastic Deterministic Decision Rule usage
//     std::cout << "\n----- Usage : StochasticDecisionRule ( sdm/core/action/stochastic_decision_rule.hpp ) ---------" << std::endl;

//     StochasticDecisionRule stoch_dr;

//     stoch_dr.setProbability(state_3, action_2, 0.5);
//     stoch_dr.setProbability(state_3, action_1, 0.5);
//     stoch_dr.setProbability(state_5, action_4, 0.8);
//     stoch_dr.setProbability(state_5, action_2, 0.2);

//     // Displays
//     std::cout << "\n#> Stochastic Decision Rule\n"
//               << stoch_dr << std::endl;

//     std::cout << "#> With o=5, u=4 --> a(u | o) = " << stoch_dr.getProbability(state_5, action_4) << std::endl;
//     std::cout << "#> With o=5 --> a(u | o) = " << stoch_dr.getProbabilities(state_5) << std::endl;


//     return 0;
// } // END main