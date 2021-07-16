/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/base_item.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>
#include <sdm/core/dynamics/tabular_observation_dynamics.hpp>
#include <sdm/world/networked_distributed_pomdp.hpp>

///////////////////////////////////////////////////////////////////////////////
//  AST processing
///////////////////////////////////////////////////////////////////////////////
namespace sdm
{
  namespace encoder
  {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // NetworkDistributed encoder
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    struct ndpomdp_encoder
    {

      number horizon, num_agents;
      std::shared_ptr<DiscreteSpace> state_space;
      std::shared_ptr<MultiDiscreteSpace> action_space, obs_space;
      std::shared_ptr<TabularReward> rewards;
      std::shared_ptr<TabularStateDynamics> state_dynamics;
      std::shared_ptr<TabularObservationDynamics> obs_dynamics;
      std::shared_ptr<DiscreteDistribution<std::shared_ptr<State>>> start_distribution;

      std::shared_ptr<sdm::NetworkedDistributedPOMDP> operator()(std::string filename)
      {
        std::ifstream input_file;
        input_file.open(filename.c_str());
        if (input_file.is_open())
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
              this->horizon = this->parse_horizon(line);
            }
            else if (line.find("NumOfAgents") != std::string::npos)
            {
              // Number of agents
              index = line.find("=");
              this->num_agents = std::stoi(line.substr(index + 1));
            }
            else if (line.find("NumOfActions") != std::string::npos)
            {
              // Action Space
              intermediate.clear();
              std::vector<std::shared_ptr<DiscreteSpace>> vector_of_indiv_action_space;
              index = line.find("=");
              contenu = line.substr(index + 1);
              std::stringstream check(contenu);
              while (std::getline(check, useless, ':'))
              {
                intermediate.push_back(useless);
              }
              number num_max_of_actions = 0;
              for (number agent_id = 0; agent_id < this->num_agents; agent_id++)
              {
                number num_action_of_agent_i = std::stoi(intermediate[agent_id]);
                vector_of_indiv_action_space.push_back(std::make_shared<DiscreteSpace>(num_action_of_agent_i));
                if (num_action_of_agent_i > num_max_of_actions)
                {
                  num_max_of_actions = num_action_of_agent_i;
                }
              }
              // this->maxActions = num_max_of_actions;
              this->action_space = std::make_shared<MultiDiscreteSpace>(vector_of_indiv_action_space);
              assert(action_space->getNumSpaces() == this->num_agents);
            }
            else if (line.find("NumOfStates") != std::string::npos)
            {
              // State Space
              index = line.find("=");
              number num_states = std::stoi(line.substr(index + 1));
              this->state_space = std::make_shared<DiscreteSpace>(num_states);
            }
            else if (line.find("NumOfObservations") != std::string::npos)
            {
              // Observation Space
              index = line.find("=");
              number num_observations = std::stoi(line.substr(index + 1));
              std::vector<number> vector_num_observations(this->num_agents, num_observations);
              this->obs_space = std::make_shared<MultiDiscreteSpace>(vector_num_observations);
            }
            else if (line.find("StartingBelief") != std::string::npos)
            {
              this->start_distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();

              intermediate.clear();
              for (const auto &state : *this->state_space)
              {
                std::getline(input_file, line);
                this->start_distribution->setProbability(state, std::stod(line))
              }
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
                number agent_id = std::stoi(intermediate2[0]);
                // Get reward
                double val = std::stod(intermediate[1]);
                // Set reward function of agent 'i'
                this->n[agent_id].rewardFunction[x + ":" + u] = val;
                std::getline(input_file, line);
              }

              rewards = std::make_shared<Reward>(action_space->getNumItems(), state_space->getNumItems());
              for (const auto &state : *state_space)
              {
                for (const auto &joint_action : *action_space)
                {
                  double joint_reward = 0;
                  for (number agent_id1 = 0; agent_id1 < this->getNumAgents(); agent_id1++)
                  {
                    for (number agent_id2 = 0; agent_id2 < this->getNumAgents(); agent_id2++)
                    {
                      joint_reward += this->getRewardF(state, agent_id1, agent_id2, joint_action[agent_id1], joint_action[agent_id2]);
                      this->reward_function_->setReward(state, joint_action, joint_reward);
                    }
                  }
                }
              }
            }
            else if (line.find("Transitions") != std::string::npos)
            {
              this->state_dynamics = std::make_shared<TabularStateDynamics>();

              // State Dynamics
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
                // Parse state id
                number state = std::stoi(intermediate[0]);
                // Parse next state id
                number next_state = std::stoi(intermediate[1]);
                // Parse tranisition probability
                double prob = std::stod(intermediate[2]);

                // Store transition probability if not zero
                if (prob > 0.0000001)
                {
                  for (const auto &action : *this->action_space)
                  {
                    this->state_dynamics->setTransitionProbability(state, action, next_state);
                  }
                }
                std::getline(input_file, line);
              }
            }
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
              number agent_id = std::stoi(intermediate[0]);
              number state = std::stoi(intermediate[1]);
              number iaction = std::stoi(intermediate[2]);
              number iobservation = std::stoi(intermediate[3]);
              double prob = std::stod(intermediate[4]);
              if (prob > 0.00001)
              {
                this->obs_dynamics_->setObservationProbability(state_space->getItem(), next_state, joint_obs, proba);
                this->observationSuccessor[std::to_string(id) + ":" + u + ":" + x].insert(std::stoi(z));
              }
              this->observationsmatrix[x + ":" + u + ":" + z] = prob;
              this->n[id].observationFunction[x + ":" + u + ":" + z] = prob;

              std::getline(input_file, line);
            }

            // Init ObservationDynamics
            this->getObsDynamics()->initDynamics(this->getActionSpace()->getNumItems(), this->getObsSpace()->getNumItems(), this->getStateSpace()->getNumItems());
            double proba, dynamics_proba;
            for (const auto &next_state : *this->state_space)
            {
              for (const auto &joint_action : *this->action_space)
              {
                for (const auto &joint_obs : *this->observation_space)
                {
                  proba = 1.;
                  for (number agent_id = 0; agent_id < this->getNumAgents(); agent_id++)
                  {
                    proba *= this->n[agent_id].observationFunction[std::to_string(next_state) + ":" + std::to_string(joint_action.get(agent_id)) + ":" + std::to_string(joint_obs.get(agent_id))];
                    // std::cout << "2 - agent_id=" << agent_id << " proba=" << proba << " joint_action=" << joint_action << " next_state=" << next_state << " joint_obs=" << joint_obs << std::endl;
                  }
                  this->obs_dynamics_->setObservationProbability(joint_action, next_state, joint_obs, proba);
                }
              }
            }
          }
        }
        else
        {
          throw std::string(" Impossible to find instance file ");
        }

        auto parsed_model = std::make_shared<sdm::NetworkedDistributedPOMDP>(state_space, action_space, obs_space, rewards, state_dynamics, obs_dynamics, start_distribution, 0, ast.discount_param, (Criterion)(ast.value_param == "reward"));

        return parsed_model;
      }

      number parser_horizon(std::string &line)
      {
        // Horizon
        index = line.find("=");
        return std::stoi(line.substr(index + 1));
      }
    };

  } // namespace ast
} // namespace sdm
