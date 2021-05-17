#include <sdm/world/networked_distributed_pomdp.hpp>

namespace sdm
{

    NetworkedDistributedPOMDP::NetworkedDistributedPOMDP(std::string filename)
    {
        input_file.open(filename.c_str());
        if (input_file.is_open())
        {
            this->getData(filename);
            this->discount_ = 1.0;
            this->setFileName(filename);
            this->setupDynamicsGenerator();
        }
        else
        {
            throw std::string(" Impossible to find instance file ");
        }
    }

    void NetworkedDistributedPOMDP::setupDynamicsGenerator()
    {
        DiscreteDecPOMDP::setupDynamicsGenerator();
        agent i;
        action a;
        state x, y;
        observation zi;

        for (x = 0; x < this->getNumStates(); ++x)
        {
            std::vector<double> v;
            for (y = 0; y < this->getNumStates(); ++y)
                v.push_back(this->getP(x, 0, y));
            this->ndpomdp_dynamics_generator.emplace(common::getState(x), std::discrete_distribution<number>(v.begin(), v.end()));

            for (i = 0; i < this->getNumAgents(); ++i)
            {
                for (a = 0; a < this->getNumActions(i); ++a)
                {
                    std::vector<double> w;
                    for (zi = 0; zi < this->getNumObservations(i); ++zi)
                        w.push_back(this->getObservation(i, a, x, zi));
                    this->ndpomdp_dynamics_generator.emplace(common::getAgentActionState(i, a, x), std::discrete_distribution<number>(w.begin(), w.end()));
                }
            }
        }
    }

    std::tuple<std::vector<double>, number, number> NetworkedDistributedPOMDP::getDynamicsGenerator(number state, number jaction)
    {
        number y = this->ndpomdp_dynamics_generator[common::getState(state)](common::global_urng());
        std::vector<observation> jobservation;
        std::vector<number> vect_jaction = this->getActionSpace()->single2joint(jaction);
        for (int ag = 0; ag < this->getNumAgents(); ag++)
        {
            number iaction = vect_jaction[ag];
            jobservation.push_back(this->ndpomdp_dynamics_generator[common::getAgentActionState(ag, iaction, y)](common::global_urng()));
        }

        double tot_rew = 0;
        for (auto nodes_pair : this->getUniqueValidNeighbors())
        {
            number first_ag = nodes_pair.first;
            number second_ag = nodes_pair.second;
            tot_rew += this->getRewardF(state, first_ag, second_ag, vect_jaction[first_ag], vect_jaction[second_ag]);
        }

        std::vector<double> v_rew(this->getNumAgents(), tot_rew);
        return std::make_tuple(v_rew, this->getObsSpace()->joint2single(jobservation), y);
    }

    std::tuple<state, observation, observation> NetworkedDistributedPOMDP::getDynamicsGenerator(state x, agent i, action ui, agent j, action uj)
    {
        state y = this->ndpomdp_dynamics_generator[common::getState(x)](common::global_urng());
        observation zi = this->ndpomdp_dynamics_generator[common::getAgentActionState(i, ui, y)](common::global_urng());
        observation zj = this->ndpomdp_dynamics_generator[common::getAgentActionState(j, uj, y)](common::global_urng());
        return std::make_tuple(y, zi, zj);
    }

    state NetworkedDistributedPOMDP::getPGenerator(state x)
    {
        return this->ndpomdp_dynamics_generator[common::getState(x)](common::global_urng());
    }

    observation NetworkedDistributedPOMDP::getQGenerator(agent i, action u, state y)
    {
        return this->ndpomdp_dynamics_generator[common::getAgentActionState(i, u, y)](common::global_urng());
    }

    state NetworkedDistributedPOMDP::init()
    {
        this->internal_state_ = this->getStartDistrib()(sdm::common::global_urng());
        return this->internal_state_;
    }

    number NetworkedDistributedPOMDP::getNumStates() const
    {
        return this->getStateSpace()->getNumItems();
    }

    number NetworkedDistributedPOMDP::getNumObservations(number agent) const
    {
        return this->getObsSpace()->getSpace(agent)->getNumItems();
    }

    number NetworkedDistributedPOMDP::getNumActions(number agent) const
    {
        return this->getActionSpace()->getSpace(agent)->getNumItems();
    }

    std::vector<number> NetworkedDistributedPOMDP::getNumActions() const
    {
        std::vector<number> v;
        for (number i = 0; i < this->getActionSpace()->getNumSpaces(); i++)
        {
            v.push_back(this->getNumActions(i));
        }
        return v;
    }

    void NetworkedDistributedPOMDP::execute(action, feedback *)
    {
        return;
    }

    void NetworkedDistributedPOMDP::getData(std::string)
    {
        std::stringstream lineStream;
        std::string line, contenu, useless, useless2;
        std::vector<std::string> intermediate, intermediate2;

        size_t index;
        // int found;
        while (std::getline(input_file, line))
        {
            if (line.find("TimeHorizon") != std::string::npos)
            {
                // Horizon
                index = line.find("=");
                this->setPlanningHorizon(std::stoi(line.substr(index + 1)));
            }
            else if (line.find("NumOfAgents") != std::string::npos)
            {
                // Number of agents
                index = line.find("=");
                this->num_agents_ = std::stoi(line.substr(index + 1));
            }
            else if (line.find("NumOfActions") != std::string::npos)
            {
                // Action Space
                intermediate.clear();
                std::vector<DiscreteSpace<number>> vector_of_indiv_action_space;
                index = line.find("=");
                contenu = line.substr(index + 1);
                std::stringstream check(contenu);
                while (std::getline(check, useless, ':'))
                {
                    intermediate.push_back(useless);
                }
                number num_max_of_actions = 0;
                for (number agent_id = 0; agent_id < this->num_agents_; agent_id++)
                {
                    number num_action_of_agent_i = std::stoi(intermediate[agent_id]);
                    vector_of_indiv_action_space.push_back(DiscreteSpace<number>(num_action_of_agent_i));
                    if (num_action_of_agent_i > num_max_of_actions)
                    {
                        num_max_of_actions = num_action_of_agent_i;
                    }
                }
                this->maxActions = num_max_of_actions;
                this->action_space_ = std::make_shared<MultiDiscreteSpace<number>>(vector_of_indiv_action_space);
                assert(this->action_space_->getNumSpaces() == this->num_agents_);
            }
            else if (line.find("NumOfNodes") != std::string::npos)
            {
                intermediate.clear();
                this->nodes = new agent[this->getNumAgents()];
                index = line.find("=");
                contenu = line.substr(index + 1);
                std::stringstream check(contenu);
                while (std::getline(check, useless, ':'))
                {
                    intermediate.push_back(useless);
                }
                for (agent k = 0; k < this->getNumAgents(); k++)
                {
                    this->nodes[k] = std::stoi(intermediate[k]);
                }
            }
            else if (line.find("NumOfStates") != std::string::npos)
            {
                // State Space
                index = line.find("=");
                this->state_space_ = std::make_shared<DiscreteSpace<number>>(std::stoi(line.substr(index + 1)));
            }
            else if (line.find("NumOfObservations") != std::string::npos)
            {
                // Observation Space
                index = line.find("=");
                number num_observations = std::stoi(line.substr(index + 1));
                std::vector<number> vector_num_observations(this->getNumAgents(), num_observations);
                this->obs_space_ = std::make_shared<MultiDiscreteSpace<number>>(vector_num_observations);
            }
            else if (line.find("Network") != std::string::npos)
            {
                //
                intermediate.clear();
                agent counter = 0;
                this->n = new Node[this->getNumAgents()];
                std::unordered_set<agent> neighbors;
                while (counter < this->getNumAgents())
                {
                    intermediate.clear();
                    std::getline(input_file, line);
                    std::stringstream check(line);
                    while (std::getline(check, useless, ' '))
                    {
                        intermediate.push_back(useless);
                    }
                    agent col = 0;
                    neighbors.clear();
                    while (col < this->getNumAgents())
                    {
                        int isConnected = std::stoi(intermediate[col]);
                        if (isConnected == 1)
                            neighbors.insert(col);
                        col++;
                    }
                    this->n[counter] = Node(counter, neighbors);
                    counter++;
                }
            }

            else if (line.find("StartingBelief") != std::string::npos)
            {
                intermediate.clear();
                std::vector<number> initial_belief = {};
                for (number x = 0; x < this->getNumStates(); ++x)
                {
                    std::getline(input_file, line);
                    initial_belief.push_back(std::stod(line));
                }
                this->start_distrib_ = std::discrete_distribution<number>(initial_belief.begin(), initial_belief.end());
            }

            else if (line.find("Reward") != std::string::npos)
            {
                // Reward function
                // les actions s'ecrivent sour la forme a_0 a_1 a_2 a_3 a_4, ou a_i est remplac� par x lorsque l'agent i n'est pas actif
                // sinon a_i appartient � une valeur comprise entre 0 et le nombre d'action de l'agent i ---  moins 1 (non?).
                std::getline(input_file, line);
                while (line.find("/*") == std::string::npos)
                {
                    intermediate.clear();
                    intermediate2.clear();
                    std::stringstream check(line);
                    while (std::getline(check, useless, ' '))
                    {
                        intermediate.push_back(useless);
                    }
                    std::string key = intermediate[0];
                    std::stringstream check1(key);
                    while (std::getline(check1, useless2, ':'))
                    {
                        intermediate2.push_back(useless2);
                    }
                    // Get state
                    std::string x = intermediate2[1];
                    // Get joint action
                    std::string u = intermediate2[2];
                    // Get agent ID
                    agent id = std::stoi(intermediate2[0]);
                    // Get reward
                    double val = std::stod(intermediate[1]);
                    // Set reward function of agent 'i'
                    this->n[id].rewardFunction[x + ":" + u] = val;
                    std::getline(input_file, line);
                }

                this->reward_function_ = std::make_shared<Reward>(this->getActionSpace()->getNumItems(), this->getStateSpace()->getNumItems());
                for (const auto &state : this->getStateSpace()->getAll())
                {
                    for (const auto &joint_action : this->getActionSpace()->getAll())
                    {
                        double joint_reward = 0;
                        for (number agent_id1 = 0; agent_id1 < this->getNumAgents(); agent_id1++)
                        {
                            for (number agent_id2 = 0; agent_id2 < this->getNumAgents(); agent_id2++)
                            {
                                joint_reward += this->getRewardF(state, agent_id1, agent_id2, joint_action[agent_id1], joint_action[agent_id2]);
                                this->reward_function_->setReward(state, this->getActionSpace()->joint2single(joint_action), joint_reward);
                            }
                        }
                    }
                }
            }
            else if (line.find("Transitions") != std::string::npos)
            {
                // State Dynamics
                std::map<std::string, double> transitionFunction;

                std::vector<Matrix> transition_matrices;
                transition_matrices.push_back(Matrix(this->getNumStates(), this->getNumStates()));

                this->stateSuccessor = new std::unordered_set<state>[this->getNumStates()];

                std::getline(input_file, line);
                while (line.find("/*") == std::string::npos)
                {
                    intermediate.clear();
                    intermediate2.clear();
                    std::stringstream check(line);
                    while (std::getline(check, useless, ' '))
                    {
                        intermediate.push_back(useless);
                    }
                    std::string key1 = intermediate[0];
                    std::string key2 = intermediate[1];
                    double prob = std::stod(intermediate[2]);
                    transitionFunction[key1 + ":" + key2] = prob;

                    // Setup State Transition Dynamics
                    transition_matrices[0](std::stoi(key1), std::stoi(key2)) = prob;

                    // Set successors
                    if (prob > 0.0000001)
                        this->stateSuccessor[std::stoi(key1)].insert(std::stoi(key2));
                    std::getline(input_file, line);
                }
                // Set transition for each agent
                for (agent id = 0; id < this->getNumAgents(); id++)
                {
                    this->n[id].transitionFunction = transitionFunction;
                }

                // Setup state transition (DiscreteDecPOMDP format)
                for (number joint_action = 1; joint_action < this->getActionSpace()->getNumItems(); ++joint_action)
                {
                    transition_matrices.push_back(transition_matrices[0]);
                }
                this->state_dynamics_->setTransitions(transition_matrices);
            }
            else if (line.find("Observations") != std::string::npos)
            {
                // Observation Dynamics
                std::getline(input_file, line);
                while (line.find("/*") == std::string::npos)
                {
                    intermediate.clear();
                    intermediate2.clear();
                    std::stringstream check(line);
                    while (std::getline(check, useless, ' '))
                    {
                        intermediate.push_back(useless);
                    }
                    agent id = std::stoi(intermediate[0]);
                    std::string x = intermediate[1];
                    std::string z = intermediate[3];
                    std::string u = intermediate[2];
                    double prob = std::stod(intermediate[4]);
                    if (prob > 0.00001)
                        this->observationSuccessor[std::to_string(id) + ":" + u + ":" + x].insert(std::stoi(z));
                    this->observationsmatrix[x + ":" + u + ":" + z] = prob;
                    this->n[id].observationFunction[x + ":" + u + ":" + z] = prob;

                    std::getline(input_file, line);
                }

                // Init ObservationDynamics
                this->getObsDynamics()->initDynamics(this->getActionSpace()->getNumItems(), this->getObsSpace()->getNumItems(), this->getStateSpace()->getNumItems());
                double proba, dynamics_proba;
                for (const auto &next_state : this->getStateSpace()->getAll())
                {
                    for (const auto &joint_action : this->getActionSpace()->getAll())
                    {
                        auto joint_action_index = this->getActionSpace()->joint2single(joint_action);
                        for (const auto &joint_obs : this->getObsSpace()->getAll())
                        {
                            auto joint_obs_index = this->getObsSpace()->joint2single(joint_obs);
                            proba = 1.;
                            for (number agent_id = 0; agent_id < this->getNumAgents(); agent_id++)
                            {
                                proba *= this->n[agent_id].observationFunction[std::to_string(next_state) + ":" + std::to_string(joint_action.get(agent_id)) + ":" + std::to_string(joint_obs.get(agent_id))];
                                // std::cout << "2 - agent_id=" << agent_id << " proba=" << proba << " joint_action=" << joint_action << " next_state=" << next_state << " joint_obs=" << joint_obs << std::endl;
                                this->obs_dynamics_->setObservationProbability(joint_action_index, joint_obs_index, next_state, proba);
                            }
                            for (auto &state : this->getStateSpace()->getAll())
                            {
                                dynamics_proba = this->getStateDynamics()->getTransitionProbability(state, joint_action_index, next_state) * this->obs_dynamics_->getObservationProbability(state, joint_action_index, joint_obs_index, next_state);
                                this->obs_dynamics_->setDynamics(state, joint_action_index, joint_obs_index, next_state, dynamics_proba);
                            }
                        }
                    }
                }
            }
        }
    }

    NetworkedDistributedPOMDP::Node::Node(agent id, std::unordered_set<agent> neighbors)
    {
        this->id = id;
        this->neighbors = neighbors;
    }

    NetworkedDistributedPOMDP::Node::Node() {}

    std::vector<std::pair<number, number>> NetworkedDistributedPOMDP::getUniqueValidNeighbors()
    {
        std::vector<std::pair<number, number>> valid_neighbors;
        for (number ag = 0; ag < this->getNumAgents(); ag++)
        {
            for (number ag2 = 0; ag2 < ag; ag2++)
            {
                if (this->n[ag].neighbors.count(ag2) > 0)
                {
                    valid_neighbors.push_back({ag, ag2});
                }
            }
        }
        return valid_neighbors;
    }

    void NetworkedDistributedPOMDP::createDAG()
    {
        this->root = 1;
        this->n[root].parent = -1;
        this->n[1].children.push_back(0);
        this->n[0].parent = 1;
        this->n[2].parent = 1;
        this->n[1].children.push_back(2);
        this->n[2].children.push_back(3);
        this->n[3].parent = 2;
        this->n[3].children.push_back(4);
        this->n[4].parent = 3;
    }

    void NetworkedDistributedPOMDP::printDAG(agent root)
    {
        std::cout << root << "[";
        for (agent i = 0; i < this->n[root].children.size(); i++)
        {
            agent childi = this->n[root].children[i];
            if (i == n[root].children.size() - 1)
            {
                std::cout << childi;
            }
            else
            {
                std::cout << childi << ",";
            }
        }
        std::cout << "]";
    }

    double NetworkedDistributedPOMDP::getRewardF(state x, agent id1, agent id2, action u1, action u2)
    {
        std::string s1 = std::to_string(x) + ":";
        std::string s2 = std::to_string(x) + ":";
        std::string s3 = "x:";
        for (agent i = 0; i < this->getNumAgents(); i++)
        {
            if (i == id1)
            {
                s1 = s1 + std::to_string(u1);
                s2 = s2 + std::to_string(u1);
                s3 = s3 + std::to_string(u1);
            }
            else if (i == id2)
            {
                s1 = s1 + std::to_string(u2);
                s2 = s2 + "x";
                s3 = s3 + "x";
            }
            else
            {
                s1 = s1 + "x";
                s2 = s2 + "x";
                s3 = s3 + "x";
            }
        }

        if (this->n[id1].rewardFunction.count(s3) == 1)
            return this->n[id1].rewardFunction[s3];

        if (this->n[id1].rewardFunction.count(s2) == 1)
            return this->n[id1].rewardFunction[s2];

        if (this->n[id1].rewardFunction.count(s1) == 1)
            return this->n[id1].rewardFunction[s1];

        return -5;
        // return 0;
    }

    double NetworkedDistributedPOMDP::getInitialBelief(state x)
    {
        return this->getStartDistrib().probabilities()[x];
    }

    double NetworkedDistributedPOMDP::getP(state x, action, state y)
    {
        std::string sx = std::to_string(x);
        std::string sy = std::to_string(y);
        std::string key = sx + ":" + sy;
        return this->n[0].transitionFunction[key];
    }

    double NetworkedDistributedPOMDP::getObservation(agent id, action u, state y, observation z)
    {
        return this->n[id].observationFunction[std::to_string(y) + ":" + std::to_string(u) + ":" + std::to_string(z)];
    }

    double NetworkedDistributedPOMDP::getQ(state x, agent id1, action u1, observation z1, agent id2, action u2, observation z2)
    {
        // std::string first = std::to_string(x) + ":" + std::to_string(u1) + ":" + std::to_string(z1);
        // std::string second = std::to_string(x) + ":" + std::to_string(u2) + ":" + std::to_string(z2);
        double val1 = this->getObservation(id1, u1, x, z1); //this->n[id1].observationFunction[first];
        double val2 = this->getObservation(id2, u2, x, z2); //this->n[id2].observationFunction[second];
        return val1 * val2;
    }

    std::unordered_set<state> NetworkedDistributedPOMDP::getStateSuccessor(state x)
    {
        return this->stateSuccessor[x];
    }

    std::unordered_set<observation> NetworkedDistributedPOMDP::getObservationSuccessor(agent id, action u, state x)
    {
        return this->observationSuccessor[std::to_string(id) + ":" + std::to_string(u) + ":" + std::to_string(x)];
    }

} // namespace sdm
