/**
 * @file belief_state_graph.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 05/05/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/struct/graph.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/public/boost_serializable.hpp>

#include <sdm/world/base/mdp_interface.hpp>

namespace sdm
{

    class POMDPInterface;

    /**
     * @brief A graph that keep all beliefs
     * 
     */
    class StateGraph : public Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>,
                       virtual public State,
                       public BoostSerializable<StateGraph>
    {
    public:
        using TransitionFunction = Pair<std::shared_ptr<State>, double>(const std::shared_ptr<MDPInterface> &,
                                                                        const std::shared_ptr<State> &,
                                                                        const std::shared_ptr<Action> &,
                                                                        const std::shared_ptr<Observation> &,
                                                                        number);

        StateGraph();
        StateGraph(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba);
        StateGraph(const std::shared_ptr<State> &data);

        /**
         * @brief Construct a new belief 
         * 
         * @param predecessor 
         * @param belief 
         */
        StateGraph(const std::shared_ptr<StateGraph> &predecessor, const std::shared_ptr<State> &belief);

        void initialize();

        /**
         * @brief Get the node associated to a given belief.
         * 
         * @param belief a specific belief
         * @return the address of the node 
         */
        std::shared_ptr<StateGraph> getNode(const std::shared_ptr<State> &belief);

        /**
         * @brief Get the probability of associated belief.
         * 
         * @param action 
         * @param observation 
         * @param belief 
         * @return the corresponding probability 
         */
        double getProbability(const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation) const;

        /**
         * @brief Expands the graph
         * 
         * @param action the action
         * @param observation the observation
         * @param backup if true, we store the expanded belief in the graph.
         * @return the next belief
         */
        std::shared_ptr<StateGraph> next(TransitionFunction transition_function, const std::shared_ptr<MDPInterface> &pomdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t, bool backup = true);

        /**
         * @brief Return a 
         */
        std::string str() const;

        /**
         * @brief Return a the pointer on this object
         */
        std::shared_ptr<StateGraph> getptr();

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);

        friend std::ostream &operator<<(std::ostream &os, StateGraph &belief_state_graph)
        {
            os << belief_state_graph.str();
            return os;
        }

    protected:
        /**
         * @brief A pointer on the bag which associated a belief to a precise pointer
         */
        // std::shared_ptr<std::unordered_map<Belief, std::shared_ptr<State>>> belief_pointer;

        /**
         * @brief A pointer on the bag containing all nodes.
         */
        std::shared_ptr<std::unordered_map<std::shared_ptr<State>, std::shared_ptr<StateGraph>>> belief_space;

        /**
         * @brief A pointer on the bag containing all nodes.
         */
        RecursiveMap<std::shared_ptr<Action>, std::shared_ptr<Observation>, double> belief_probability;

        virtual std::shared_ptr<State> exist(const std::shared_ptr<State> &);
    };

} // namespace sdm
