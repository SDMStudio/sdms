#include <sdm/world/ndpomdp.hpp>

namespace sdm
{

    NDPOMDP::NDPOMDP(std::string filename)
    {
        input_file.open(filename.c_str());
        if (input_file.is_open())
        {
            this->getData(filename);
            this->setupDynamicsGenerator();
        }

        else
            throw std::string(" Impossible to find instance file ");
    }

    void NDPOMDP::setupDynamicsGenerator()
    {
        agent i;
        action a;
        state x, y;
        observation zi;

        for (x = 0; x < this->getNumStates(); ++x)
        {
            std::vector<double> v;
            for (y = 0; y < this->getNumStates(); ++y)
                v.push_back(this->getP(x, 0, y));
            this->dynamics_generator.emplace(common::getState(x), std::discrete_distribution<number>(v.begin(), v.end()));

            for (i = 0; i < this->getNumAgents(); ++i)
            {
                for (a = 0; a < this->getNumActions(i); ++a)
                {
                    std::vector<double> w;
                    for (zi = 0; zi < this->getNumObservations(i); ++zi)
                        w.push_back(this->getObservation(i, a, x, zi));
                    this->dynamics_generator.emplace(common::getAgentActionState(i, a, x), std::discrete_distribution<number>(w.begin(), w.end()));
                }
            }
        }

        std::vector<double> v;
        for (x = 0; x < this->getNumStates(); ++x)
            v.push_back(this->getInitialBelief(x));
        this->start_generator = std::discrete_distribution<number>(v.begin(), v.end());
    }

    std::tuple<std::vector<double>, number, number> NDPOMDP::getDynamicsGenerator(number state, number jaction)
    {
        number y = this->dynamics_generator[common::getState(state)](common::global_urng());
        std::vector<observation> jobservation;
        std::vector<number> vect_jaction = this->getActionSpace().single2joint(jaction);
        for (int ag = 0; ag < this->getNumAgents(); ag++)
        {
            number iaction = vect_jaction[ag];
            jobservation.push_back(this->dynamics_generator[common::getAgentActionState(ag, iaction, y)](common::global_urng()));
        }

        double tot_rew = 0;
        for (auto nodes_pair : this->getUniqueValidNeighbors())
        {
            number first_ag = nodes_pair.first;
            number second_ag = nodes_pair.second;
            tot_rew += this->getReward(state, first_ag, second_ag, vect_jaction[first_ag], vect_jaction[second_ag]);
        }

        std::vector<double> v_rew(this->getNumAgents(), tot_rew);
        return std::make_tuple(v_rew, this->obs_space_.joint2single(jobservation), y);
    }

    std::tuple<state, observation, observation> NDPOMDP::getDynamicsGenerator(state x, agent i, action ui, agent j, action uj)
    {
        state y = this->dynamics_generator[common::getState(x)](common::global_urng());
        observation zi = this->dynamics_generator[common::getAgentActionState(i, ui, y)](common::global_urng());
        observation zj = this->dynamics_generator[common::getAgentActionState(j, uj, y)](common::global_urng());
        return std::make_tuple(y, zi, zj);
    }

    state NDPOMDP::getPGenerator(state x)
    {
        return this->dynamics_generator[common::getState(x)](common::global_urng());
    }

    observation NDPOMDP::getQGenerator(agent i, action u, state y)
    {
        return this->dynamics_generator[common::getAgentActionState(i, u, y)](common::global_urng());
    }

    state NDPOMDP::init()
    {
        this->internal_state_ = this->start_generator(sdm::common::global_urng());
        return this->internal_state_;
    }

    void NDPOMDP::setDiscount(double discount)
    {
        this->discount = discount;
    }

    double NDPOMDP::getDiscount()
    {
        return this->discount;
    }

    number NDPOMDP::getNumStates() const
    {
        return this->state_space_.getNumItems();
    }

    number NDPOMDP::getNumObservations(number agent) const
    {
        return this->obs_space_.getSpace(agent)->getNumItems();
    }

    number NDPOMDP::getNumAgents() const
    {
        return this->agent_space_.getNumItems();
    }

    const MultiDiscreteSpace<number> &NDPOMDP::getActionSpace() const
    {
        return this->act_space_;
    }

    number NDPOMDP::getNumJActions() const
    {
        return this->act_space_.getNumJointItems();
    }

    number NDPOMDP::getNumActions(number agent) const
    {
        return this->getActionSpace().getSpace(agent)->getNumItems();
    }

    std::vector<number> NDPOMDP::getNumActions() const
    {
        std::vector<number> v;
        for (number i = 0; i < this->getActionSpace().getNumSpaces(); i++)
        {
            v.push_back(this->getNumActions(i));
        }
        return v;
    }

    void NDPOMDP::execute(action, feedback *)
    {
        return;
    }

    void NDPOMDP::getData(std::string)
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
                index = line.find("=");
                this->timeHorizon = std::stoi(line.substr(index + 1));
            }
            else if (line.find("NumOfAgents") != std::string::npos)
            {
                index = line.find("=");

                std::vector<number> vv_tmp(std::stoi(line.substr(index + 1)));
                std::iota(vv_tmp.begin(), vv_tmp.end(), 0);
                this->agent_space_ = DiscreteSpace<number>(vv_tmp);
                std::cout << "Agent Space parsed : \n"
                          << this->agent_space_ << std::endl;
            }
            else if (line.find("NumOfActions") != std::string::npos)
            {
                intermediate.clear();
                std::vector<DiscreteSpace<number>> v_act_space;
                index = line.find("=");
                contenu = line.substr(index + 1);
                std::stringstream check(contenu);
                while (std::getline(check, useless, ':'))
                {
                    intermediate.push_back(useless);
                }
                action max = 0;
                for (agent k = 0; k < this->getNumAgents(); k++)
                {
                    number n_a = std::stoi(intermediate[k]);
                    std::vector<number> vv_tmp(n_a);
                    std::iota(vv_tmp.begin(), vv_tmp.end(), 0);
                    v_act_space.push_back(DiscreteSpace<number>(vv_tmp));
                    if (n_a > max)
                    {
                        max = n_a;
                    }
                }
                this->maxActions = max;
                this->act_space_ = MultiDiscreteSpace<number>(v_act_space);
                std::cout << "Action Space parsed : \n"
                          << this->act_space_ << std::endl;
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
                index = line.find("=");
                std::vector<number> vv_tmp(std::stoi(line.substr(index + 1)));
                std::iota(vv_tmp.begin(), vv_tmp.end(), 0);
                this->state_space_ = DiscreteSpace<number>(vv_tmp);
                std::cout << "State Space parsed : \n"
                          << this->state_space_ << std::endl;
            }
            else if (line.find("NumOfObservations") != std::string::npos)
            {
                index = line.find("=");
                number observations = std::stoi(line.substr(index + 1));
                std::vector<number> vv_tmp(observations);
                std::iota(vv_tmp.begin(), vv_tmp.end(), 0);
                std::vector<std::vector<number>> obs_v(this->getNumAgents(), vv_tmp);
                this->obs_space_ = MultiDiscreteSpace<number>(obs_v);
                std::cout << "Observation Space parsed : \n"
                          << this->obs_space_ << std::endl;
            }
            else if (line.find("Network") != std::string::npos)
            {
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
                state x = 0;
                intermediate.clear();
                this->startingBelief = Vector(this->getNumStates());
                while (x < this->getNumStates())
                {
                    std::getline(input_file, line);
                    this->startingBelief[x] = std::stod(line);
                    x++;
                }
            }

            else if (line.find("Reward") != std::string::npos)
            {
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
                    std::string x = intermediate2[1];
                    std::string u = intermediate2[2];
                    agent id = std::stoi(intermediate2[0]);
                    double val = std::stod(intermediate[1]);
                    this->n[id].rewardFunction[x + ":" + u] = val;
                    std::getline(input_file, line);
                }
            }
            else if (line.find("Transitions") != std::string::npos)
            {
                std::map<std::string, double> transitionFunction;
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
                    if (prob > 0.0000001)
                        this->stateSuccessor[std::stoi(key1)].insert(std::stoi(key2));
                    std::getline(input_file, line);
                }
                for (agent id = 0; id < this->getNumAgents(); id++)
                {
                    this->n[id].transitionFunction = transitionFunction;
                }
            }
            else if (line.find("Observations") != std::string::npos)
            {
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
                    std::string x = intermediate[1];
                    std::string u = intermediate[2];
                    std::string z = intermediate[3];
                    agent id = std::stoi(intermediate[0]);
                    double prob = std::stod(intermediate[4]);
                    if (prob > 0.00001)
                        this->observationSuccessor[std::to_string(id) + ":" + u + ":" + x].insert(std::stoi(z));
                    this->observationsmatrix[x + ":" + u + ":" + z] = prob;
                    this->n[id].observationFunction[x + ":" + u + ":" + z] = prob;
                    std::getline(input_file, line);
                }
            }
        }
    }

    NDPOMDP::Node::Node(agent id, std::unordered_set<agent> neighbors)
    {
        this->id = id;
        this->neighbors = neighbors;
    }

    NDPOMDP::Node::Node() {}

    std::vector<std::pair<number, number>> NDPOMDP::getUniqueValidNeighbors()
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

    void NDPOMDP::createDAG()
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

    void NDPOMDP::printDAG(agent root)
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

    double NDPOMDP::getReward(state x, agent id1, agent id2, action u1, action u2)
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

    double NDPOMDP::getInitialBelief(state x)
    {
        return this->startingBelief[x];
    }

    double NDPOMDP::getP(state x, action, state y)
    {
        std::string sx = std::to_string(x);
        std::string sy = std::to_string(y);
        std::string key = sx + ":" + sy;
        return this->n[0].transitionFunction[key];
    }

    double NDPOMDP::getObservation(agent id, action u, state y, observation z)
    {
        return this->n[id].observationFunction[std::to_string(y) + ":" + std::to_string(u) + ":" + std::to_string(z)];
    }

    double NDPOMDP::getQ(state x, agent id1, action u1, observation z1, agent id2, action u2, observation z2)
    {
        // std::string first = std::to_string(x) + ":" + std::to_string(u1) + ":" + std::to_string(z1);
        // std::string second = std::to_string(x) + ":" + std::to_string(u2) + ":" + std::to_string(z2);
        double val1 = this->getObservation(id1, u1, x, z1); //this->n[id1].observationFunction[first];
        double val2 = this->getObservation(id2, u2, x, z2); //this->n[id2].observationFunction[second];
        return val1 * val2;
    }

    std::unordered_set<state> NDPOMDP::getStateSuccessor(state x)
    {
        return this->stateSuccessor[x];
    }

    std::unordered_set<observation> NDPOMDP::getObservationSuccessor(agent id, action u, state x)
    {
        return this->observationSuccessor[std::to_string(id) + ":" + std::to_string(u) + ":" + std::to_string(x)];
    }

} // namespace sdm
