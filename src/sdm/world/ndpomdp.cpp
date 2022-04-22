// #include <sdm/world/ndpomdp.hpp>
// #include <sdm/core/state/base_state.hpp>
// #include <sdm/core/observation/base_observation.hpp>
// #include <sdm/core/action/base_action.hpp>
// #include <sdm/core/space/multi_discrete_space.hpp>

// namespace sdm
// {

//     NDPOMDP::NDPOMDP(std::string filename)
//     {
//         input_file.open(filename.c_str());
//         if (input_file.is_open())
//         {
//             this->getData(filename);
//             // this->setFileName(filename);
//             // this->setupDynamicsGenerator();
//         }
//         else
//         {
//             throw std::string(" Impossible to find instance file ");
//         }
//     }

//     std::shared_ptr<State> NDPOMDP::init()
//     {
//         this->internal_state = this->getStartDistribution()->sample();
//         return this->internal_state;
//     }

//     std::shared_ptr<MPOMDPInterface> NDPOMDP::getSubMPOMDP(const Pair<number, number> &couple_agent) const
//     {
//         return this->sub_pomdp.at(couple_agent);
//     }

//     number NDPOMDP::getNumAgents() const
//     {
//         return this->num_agents;
//     }

//     number NDPOMDP::getHorizon() const
//     {
//         return this->horizon;
//     }

//     double NDPOMDP::getDiscount(number t) const
//     {
//         return this->discount;
//     }

//     double NDPOMDP::getWeightedDiscount(number t) const
//     {
//         return pow(this->getDiscount(t), t);
//     }

//     std::shared_ptr<Distribution<std::shared_ptr<State>>> NDPOMDP::getStartDistribution() const
//     {
//         return this->start_distribution;
//     }

//     std::shared_ptr<Space> NDPOMDP::getStateSpace(number t) const
//     {
//         return this->state_space;
//     }

//     std::shared_ptr<Space> NDPOMDP::getActionSpace(number t) const
//     {
//         return this->action_space;
//     }

//     double NDPOMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
//     {
//         // VERSION 1
//         number index_state = this->getStateSpace(t)->toDiscreteSpace()->getItemIndex(state);
//         auto jaction = std::static_pointer_cast<JointAction>(action);
//         double joint_reward = 0.;
//         for (const auto couple : this->list_neighboors)
//         {
//             number idx_agent_1 = couple.first, idx_agent_2 = couple.second;
//             number idx_action_ag1 = this->getActionSpace(t)->toMultiDiscreteSpace()->getItemIndex(idx_agent_1, jaction->get(idx_agent_1));
//             number idx_action_ag2 = this->getActionSpace(t)->toMultiDiscreteSpace()->getItemIndex(idx_agent_2, jaction->get(idx_agent_2));

//             joint_reward += this->getRewardF(index_state, idx_agent_1, idx_agent_2, idx_action_ag1, idx_action_ag2);
//         }
//         return joint_reward;

//         // VERSION 2
//         // double joint_reward = 0.;
//         // auto jaction = std::static_pointer_cast<JointAction>(action);
//         // for (const auto couple : this->list_neighboors)
//         // {
//         //     JointAction action_couple({jaction->get(couple.first), jaction->get(couple.second)});
//         //     auto action_couple_addr = this->getSubMPOMDP(couple)->getActionSpace(t)->toDiscreteSpace()->getItemAddress(action_couple);
//         //     joint_reward += this->getSubMPOMDP(couple)->getReward(state, action_couple_addr, t);
//         // }
//         // return joint_reward;
//     }

//     double NDPOMDP::getMinReward(number t) const
//     {
//     }

//     double NDPOMDP::getMaxReward(number t) const
//     {
//     }

//     std::tuple<std::shared_ptr<State>, std::vector<double>, bool> NDPOMDP::step(std::shared_ptr<Action> action)
//     {
//     }

//     std::tuple<std::shared_ptr<State>, std::vector<double>, bool> NDPOMDP::step(std::shared_ptr<Action> action, bool increment_timestep)
//     {
//     }

//     double NDPOMDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &, const std::shared_ptr<State> &next_state, number t) const
//     {
//         return this->n[0].transitionFunction[this->getTransitionName(state, next_state)]
//     }

//     std::set<std::shared_ptr<State>> NDPOMDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
//     {
//         // VERSION 2
//         return this->getSubMPOMDP(this->list_neighboors.at(0))->getReachableStates(state, action, t);
//     }

//     std::shared_ptr<Space> NDPOMDP::getObservationSpace(number t) const
//     {
//         return this->observation_space;
//     }

//     std::set<std::shared_ptr<Observation>> NDPOMDP::getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
//     {
//     }

//     double NDPOMDP::getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
//     {
//     }

//     double NDPOMDP::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
//     {
//         auto jaction = std::static_pointer_cast<JointAction>(action);
//         auto jobservation = std::static_pointer_cast<JointObservation>(observation);

//         double proba = this->n[0].transitionFunction[this->getTransitionName(state, next_state)];

//         for (number agent_id = 0; agent_id < this->getNumAgents(); agent_id++)
//         {
//             proba *= this->n[agent_id].observationFunction[std::to_string(index_state) + ":" + std::to_string(jaction->get(agent_id)) + ":" + std::to_string(jobservation->get(agent_id))]
//         }
//     }

//     void NDPOMDP::setInternalState(std::shared_ptr<State> state)
//     {
//         this->internal_state = state;
//     }

//     std::shared_ptr<State> NDPOMDP::getInternalState() const
//     {
//         return this->internal_state;
//     }

//     void NDPOMDP::getData(std::string)
//     {
//         std::stringstream lineStream;
//         std::string line, contenu, useless, useless2;
//         std::vector<std::string> intermediate, intermediate2;
//         size_t index;
//         // int found;
//         while (std::getline(input_file, line))
//         {
//             if (line.find("TimeHorizon") != std::string::npos)
//             {
//                 // Horizon
//                 index = line.find("=");
//                 this->horizon = std::stoi(line.substr(index + 1));
//             }
//             else if (line.find("NumOfAgents") != std::string::npos)
//             {
//                 // Number of agents
//                 index = line.find("=");
//                 this->num_agents = std::stoi(line.substr(index + 1));
//             }
//             else if (line.find("NumOfActions") != std::string::npos)
//             {
//                 // Action Space
//                 intermediate.clear();
//                 std::vector<std::vector<std::shared_ptr<Item>>> list_items;
//                 index = line.find("=");
//                 contenu = line.substr(index + 1);
//                 std::stringstream check(contenu);
//                 while (std::getline(check, useless, ':'))
//                 {
//                     intermediate.push_back(useless);
//                 }
//                 number num_max_of_actions = 0;
//                 for (number agent_id = 0; agent_id < this->getNumAgents(); agent_id++)
//                 {
//                     list_items.push_back({});
//                     number num_action_of_agent_i = std::stoi(intermediate[agent_id]);
//                     for (number i = 0; i < num_action_of_agent_i; i++)
//                     {
//                         list_items[agent_id].push_back(std::make_shared<DiscreteAction>(i));
//                     }
//                     if (num_action_of_agent_i > num_max_of_actions)
//                     {
//                         num_max_of_actions = num_action_of_agent_i;
//                     }
//                 }
//                 // this->maxActions = num_max_of_actions;
//                 this->action_space = std::make_shared<MultiDiscreteSpace>(list_items);
//                 assert(std::static_pointer_cast<MultiDiscreteSpace>(this->action_space_)->getNumSpaces() == this->num_agents_);
//             }
//             else if (line.find("NumOfNodes") != std::string::npos)
//             {
//                 intermediate.clear();
//                 this->nodes = new number[this->getNumAgents()];
//                 index = line.find("=");
//                 contenu = line.substr(index + 1);
//                 std::stringstream check(contenu);
//                 while (std::getline(check, useless, ':'))
//                 {
//                     intermediate.push_back(useless);
//                 }
//                 for (number k = 0; k < this->getNumAgents(); k++)
//                 {
//                     this->nodes[k] = std::stoi(intermediate[k]);
//                 }
//             }
//             else if (line.find("NumOfStates") != std::string::npos)
//             {
//                 // State Space
//                 index = line.find("=");
//                 number num_states = std::stoi(line.substr(index + 1));
//                 std::vector<std::shared_ptr<Item>> list_states;
//                 for (number i = 0; i < num_states; i++)
//                 {
//                     list_states.push_back(std::make_shared<DiscreteState>(i));
//                 }
//                 this->state_space = std::make_shared<DiscreteSpace>(list_states);
//             }
//             else if (line.find("NumOfObservations") != std::string::npos)
//             {
//                 // Observation Space
//                 index = line.find("=");
//                 number num_observations = std::stoi(line.substr(index + 1));
//                 std::vector<std::shared_ptr<Item>> list_obs;
//                 for (number i = 0; i < num_observations; i++)
//                 {
//                     list_obs.push_back(std::make_shared<DiscreteObservation>(i));
//                 }
//                 std::vector<std::vector<std::shared_ptr<Item>>> all_list_obs(this->getNumAgents(), list_obs);
//                 this->observation_space = std::make_shared<MultiDiscreteSpace>(all_list_obs);
//             }
//             else if (line.find("Network") != std::string::npos)
//             {
//                 //
//                 intermediate.clear();
//                 number counter = 0;
//                 this->n = new Node[this->getNumAgents()];
//                 std::unordered_set<number> neighbors;
//                 while (counter < this->getNumAgents())
//                 {
//                     intermediate.clear();
//                     std::getline(input_file, line);
//                     std::stringstream check(line);
//                     while (std::getline(check, useless, ' '))
//                     {
//                         intermediate.push_back(useless);
//                     }
//                     number col = 0;
//                     neighbors.clear();
//                     while (col < this->getNumAgents())
//                     {
//                         int isConnected = std::stoi(intermediate[col]);
//                         if (isConnected == 1)
//                             neighbors.insert(col);
//                         col++;
//                     }
//                     this->n[counter] = Node(counter, neighbors);
//                     counter++;
//                 }
//             }
//             else if (line.find("StartingBelief") != std::string::npos)
//             {
//                 auto start_distribution_tmp = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
//                 intermediate.clear();
//                 for (const auto &state : *this->state_space)
//                 {
//                     std::getline(input_file, line);
//                     start_distribution_tmp->setProbability(state, std::stod(line));
//                 }
//                 this->start_distribution = start_distribution_tmp;
//             }
//             else if (line.find("Reward") != std::string::npos)
//             {
//                 // Reward function
//                 // les actions s'ecrivent sour la forme a_0 a_1 a_2 a_3 a_4, ou a_i est remplac� par x lorsque l'agent i n'est pas actif
//                 // sinon a_i appartient � une valeur comprise entre 0 et le nombre d'action de l'agent i ---  moins 1 (non?).
//                 std::getline(input_file, line);
//                 while (line.find("/*") == std::string::npos)
//                 {
//                     intermediate.clear();
//                     intermediate2.clear();
//                     std::stringstream check(line);
//                     while (std::getline(check, useless, ' '))
//                     {
//                         intermediate.push_back(useless);
//                     }
//                     std::string key = intermediate[0];
//                     std::stringstream check1(key);
//                     while (std::getline(check1, useless2, ':'))
//                     {
//                         intermediate2.push_back(useless2);
//                     }

//                     // Get state
//                     std::string x = intermediate2[1];
//                     // Get joint action
//                     std::string u = intermediate2[2];
//                     // Get agent ID
//                     number agent_id = std::stoi(intermediate2[0]);
//                     // Get reward
//                     double val = std::stod(intermediate[1]);

//                     // Set reward function of agent 'i'
//                     this->n[agent_id].rewardFunction[x + ":" + u] = val;
//                     std::getline(input_file, line);
//                 }
//             }
//             else if (line.find("Transitions") != std::string::npos)
//             {

//                 // State Dynamics
//                 std::map<std::string, double> transitionFunction;

//                 std::getline(input_file, line);
//                 while (line.find("/*") == std::string::npos)
//                 {
//                     intermediate.clear();
//                     intermediate2.clear();
//                     std::stringstream check(line);
//                     while (std::getline(check, useless, ' '))
//                     {
//                         intermediate.push_back(useless);
//                     }
//                     // Get address of a state
//                     std::shared_ptr<State> key1 = this->getStateSpace(0)->toDiscreteSpace()->getItem(std::stod(intermediate[0]));
//                     std::shared_ptr<State> key2 = this->getStateSpace(0)->toDiscreteSpace()->getItem(std::stod(intermediate[1]));

//                     // Proba of the transition
//                     double prob = std::stod(intermediate[2]);

//                     // Assign to the data structure
//                     transitionFunction[this->getTransitionName(key1, key2)] = prob;

//                     std::getline(input_file, line);
//                 }
//                 // Set transition for each agent
//                 for (number id = 0; id < this->getNumAgents(); id++)
//                 {
//                     this->n[id].transitionFunction = transitionFunction;
//                 }
//             }
//             else if (line.find("Observations") != std::string::npos)
//             {
//                 // Observation Dynamics
//                 std::getline(input_file, line);
//                 while (line.find("/*") == std::string::npos)
//                 {
//                     intermediate.clear();
//                     intermediate2.clear();
//                     std::stringstream check(line);
//                     while (std::getline(check, useless, ' '))
//                     {
//                         intermediate.push_back(useless);
//                     }
//                     number agent_id = std::stoi(intermediate[0]);
//                     std::string state = intermediate[1];
//                     std::string iaction = intermediate[2];
//                     std::string iobservation = intermediate[3];
//                     double prob = std::stod(intermediate[4]);
//                     if (prob > 0.00001)
//                     {
//                         this->observationSuccessor[std::to_string(agent_id) + ":" + iaction + ":" + state].insert(std::stoi(iobservation));
//                     }
//                     this->observationsmatrix[state + ":" + iaction + ":" + iobservation] = prob;
//                     this->n[agent_id].observationFunction[state + ":" + iaction + ":" + iobservation] = prob;

//                     std::getline(input_file, line);
//                 }
//             }
//         }
//     }

//     NDPOMDP::Node::Node(number agent_id, std::unordered_set<number> neighbors)
//     {
//         this->agent_id = agent_id;
//         this->neighbors = neighbors;
//     }

//     NDPOMDP::Node::Node() {}

//     std::vector<std::pair<number, number>> NDPOMDP::getUniqueValidNeighbors()
//     {
//         std::vector<std::pair<number, number>> valid_neighbors;
//         for (number ag = 0; ag < this->getNumAgents(); ag++)
//         {
//             for (number ag2 = 0; ag2 < ag; ag2++)
//             {
//                 if (this->n[ag].neighbors.count(ag2) > 0)
//                 {
//                     valid_neighbors.push_back({ag, ag2});
//                 }
//             }
//         }
//         return valid_neighbors;
//     }

//     double NDPOMDP::getRewardCouple(number x, number id1, number id2, number u1, number u2) const
//     {
//         std::string s1 = std::to_string(x) + ":";
//         std::string s2 = std::to_string(x) + ":";
//         std::string s3 = "x:";
//         for (number i = 0; i < this->getNumAgents(); i++)
//         {
//             if (i == id1)
//             {
//                 s1 = s1 + std::to_string(u1);
//                 s2 = s2 + std::to_string(u1);
//                 s3 = s3 + std::to_string(u1);
//             }
//             else if (i == id2)
//             {
//                 s1 = s1 + std::to_string(u2);
//                 s2 = s2 + "x";
//                 s3 = s3 + "x";
//             }
//             else
//             {
//                 s1 = s1 + "x";
//                 s2 = s2 + "x";
//                 s3 = s3 + "x";
//             }
//         }

//         if (this->n[id1].rewardFunction.count(s3) == 1)
//             return this->n[id1].rewardFunction[s3];

//         if (this->n[id1].rewardFunction.count(s2) == 1)
//             return this->n[id1].rewardFunction[s2];

//         if (this->n[id1].rewardFunction.count(s1) == 1)
//             return this->n[id1].rewardFunction[s1];

//         // return -5;
//         return 0;
//     }

//     double NDPOMDP::getInitialBelief(std::shared_ptr<State> x)
//     {
//         return std::static_pointer_cast<DiscreteDistribution<std::shared_ptr<State>>>(this->getStartDistribution())->getProbability(x);
//     }

//     double NDPOMDP::getObservation(number id, number u, number y, number z)
//     {
//         return this->n[id].observationFunction[std::to_string(y) + ":" + std::to_string(u) + ":" + std::to_string(z)];
//     }

//     // std::set<std::shared_ptr<Observation>> NDPOMDP::getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
//     // {
//     //     // return this->observation_dynamics_->getReachableObservations(state, action, next_state, t);
//     //     // this->observationSuccessor[std::to_string(agent_id) + ":" + iaction + ":" + state].insert(std::stoi(iobservation));
//     //     std::set<std::shared_ptr<Observation>> set_reachable;
//     //     for (const auto &obs : *this->getObservationSpace())
//     //     {
//     //         if (getObservationProbability(state, action, next_state, obs, t) > 0)
//     //         {
//     //             set_reachable.insert(obs);
//     //         }
//     //     }
//     //     return set_reachable;
//     // }

//     // double NDPOMDP::getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
//     // {

//     //     double proba = 1.;
//     //     auto idx_state = this->getStateSpace(t + 1)->toDiscreteSpace()->getItemIndex(next_state);
//     //     for (number ag = 0; ag < this->getNumAgents(); ag++)
//     //     {
//     //         auto idx_action_ag = this->getActionSpace()->toMultiDiscreteSpace()->getItemIndex(ag, std::static_pointer_cast<JointAction>(action)->get(ag));
//     //         auto idx_obs_ag = this->getObservationSpace()->toMultiDiscreteSpace()->getItemIndex(ag, std::static_pointer_cast<JointObservation>(observation)->get(ag));
//     //         proba *= this->n[ag].observationFunction[std::to_string(idx_state) + ":" + std::to_string(idx_action_ag) + ":" + std::to_string(idx_obs_ag)];
//     //     }
//     //     return proba;
//     // }

//     // std::shared_ptr<ObservationDynamicsInterface> NDPOMDP::getObservationDynamics() const
//     // {
//     //     return nullptr;
//     // }

//     // std::shared_ptr<Observation> NDPOMDP::sampleNextObservation(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
//     // {
//     //     MDP::sampleNextObservation(state, action, t);
//     //     double epsilon = std::rand() / (double(RAND_MAX)), cumul = 0.;

//     //     // Go over all observations of the lower-level agent
//     //     auto obs_space = this->getObservationSpace(t);
//     //     for (auto obs_n : *obs_space)
//     //     {

//     //         cumul += this->getObservationProbability(state, action, this->getInternalState(), obs_n, t);
//     //         if (epsilon < cumul)
//     //         {
//     //             return obs_n;
//     //         }
//     //     }
//     //     return nullptr;
//     // }

//     // double NDPOMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
//     // {

//     //     number index_state = this->getStateSpace(t)->toDiscreteSpace()->getItemIndex(state);
//     //     auto jaction = std::static_pointer_cast<JointAction>(action);
//     //     double joint_reward = 0.;
//     //     for (number agent_id1 = 0; agent_id1 < this->getNumAgents(); agent_id1++)
//     //     {
//     //         number idx_action_ag1 = this->getActionSpace(t)->toMultiDiscreteSpace()->getItemIndex(agent_id1, jaction->get(agent_id1));
//     //         for (number agent_id2 = 0; agent_id2 < this->getNumAgents(); agent_id2++)
//     //         {
//     //             number idx_action_ag2 = this->getActionSpace(t)->toMultiDiscreteSpace()->getItemIndex(agent_id2, jaction->get(agent_id2));

//     //             joint_reward += this->getRewardF(index_state, agent_id1, agent_id2, idx_action_ag1, idx_action_ag2);
//     //         }
//     //     }
//     //     return joint_reward;
//     // }

//     // double NDPOMDP::getMinReward(number) const
//     // {
//     //     return 0;
//     // }

//     // double NDPOMDP::getMaxReward(number) const
//     // {
//     //     return std::numeric_limits<double>::infinity();
//     // }

// } // namespace sdm
