#pragma once

#include <string>
#include <vector>
#include <list>
#include <fstream>
#include <iostream>
#include <sstream>
#include <map>
#include <unordered_set>
#include "math.h"
#include <random>

#include <sdm/types.hpp>
#include <sdm/common.hpp>
#include <sdm/public/world.hpp>
// #include <sdm/public/feedback.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/world/mpomdp.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>

namespace sdm
{
    class NetworkedDistributedPOMDP : public MPOMDP
    {
    private:
        std::shared_ptr<State> internal_state_;

    public:
        using agent = number;
        using observation = number;
        using state = number;
        using action = number;

        class Node
        {
        public:
            agent id;
            int parent = -2;
            double *upperBounds;
            std::vector<agent> children;
            std::unordered_set<agent> neighbors;
            std::map<std::string, double> rewardFunction;
            std::map<std::string, double> transitionFunction;
            std::map<std::string, double> observationFunction;

            Node();
            Node(agent id, std::unordered_set<agent>);
        };

        number num_agents_;

        std::map<int, int> transitionmatrix;
        std::unordered_set<state> *stateSuccessor;
        std::map<std::string, double> observationsmatrix;
        std::map<std::string, std::unordered_set<observation>> observationSuccessor;

        /**
         * @brief dynamics generator
         */
        std::unordered_map<std::string, std::discrete_distribution<number>> ndpomdp_dynamics_generator;

        Node *n;

        agent *nodes;

        agent root;

        std::ifstream input_file;

        double rMax = 45;

        NetworkedDistributedPOMDP(std::string);

        void createDAG();
        void printDAG(agent);
        virtual std::shared_ptr<State> init();
        void getData(std::string);

        std::vector<std::pair<number, number>> getUniqueValidNeighbors();

        double getInitialBelief(std::shared_ptr<State>);
        // virtual void execute(action, feedback *);

        double getRewardF(state, agent, agent, action, action);
        double getObservation(agent, action, state, observation);


        // virtual number getNumStates() const;

        // /**
        //  * @brief Get the number of observations for a specific agent
        //  *
        //  * @return number
        //  */
        // virtual number getNumObservations(number) const;

        // /**
        //  * \brief Get the number of Actions for a specific agent
        //  */
        // number getNumActions(number) const;

        // /**
        //  * \brief Get the number of Action for every agents
        //  */
        // std::vector<number> getNumActions() const;


        void setupDynamicsGenerator();

        /**
         * @fn std::tuple<double, observation, state> getDynamicsGenerator(state x, action a)
         * @param state the current state
         * @param jaction the joint action
         * @return a tuple containing reward, next_osbservation and next_state
         **/
        std::tuple<std::vector<double>, observation, state> getDynamicsGenerator(number, number);

        // std::unordered_set<state> getStateSuccessor(state);
        // std::unordered_set<observation> getObservationSuccessor(agent, action, state);
        // std::tuple<state, observation, observation> getDynamicsGenerator(state, agent, action, agent, action);


        // state getPGenerator(state);
        // observation getQGenerator(agent, action, state);
        // double getP(state, action, state);
        // double getQ(state, agent, action, observation, agent, action, observation);
    };
} // namespace sdm
