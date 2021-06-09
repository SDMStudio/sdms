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

#include <sdm/public/boost_serializable.hpp>
#include <sdm/utils/struct/graph.hpp>
#include <sdm/utils/linear_algebra/sdms_matrix.hpp>

namespace sdm
{
    /**
     * @brief A graph that keep all beliefs
     * 
     * @tparam TBeliefState 
     * @tparam TAction 
     * @tparam TObservation 
     */
    template <typename TBeliefState, typename TAction, typename TObservation>
    class BeliefStateGraph : public Graph<TBeliefState, Pair<TAction, TObservation>>,
                             public BoostSerializable<BeliefStateGraph<TBeliefState, TAction, TObservation>>
    {
    public:
        using action_type = TAction;
        using state_type = TBeliefState;

        BeliefStateGraph();
        BeliefStateGraph(const std::vector<typename TBeliefState::state_type> &list_states, const std::vector<double> &list_proba,  const std::vector<std::vector<DenseMatrix<number, number>>> &dynamics = {});
        BeliefStateGraph(const TBeliefState &data, const std::vector<std::vector<DenseMatrix<number, number>>> &dynamics = {});

        /**
         * @brief Construct a new belief 
         * 
         * @param predecessor 
         * @param belief 
         */
        BeliefStateGraph(const std::shared_ptr<BeliefStateGraph> &predecessor, const TBeliefState &belief);

        /**
         * @brief Initialize the graph of belief.
         */
        void initialize();

        /**
         * @brief Get the node associated to a given belief.
         * 
         * @param belief a specific belief
         * @return the address of the node 
         */
        std::shared_ptr<BeliefStateGraph> getNode(const TBeliefState &belief);

        /**
         * @brief Set the dynamics that is used to compute the next belief.
         * 
         * @param dynamics the matrix of state transition for each (action, observation) pair. 
         */
        void setDynamics(std::vector<std::vector<DenseMatrix<number, number>>> dynamics);

        /**
         * @brief Get the probability of associated belief.
         * 
         * @param action 
         * @param observation 
         * @param belief 
         * @return the corresponding probability 
         */
        double getProbability(const TAction &action, const TObservation &observation) const;

        /**
         * @brief Expands the graph
         * 
         * @param action the action
         * @param observation the observation
         * @param backup if true, we store the expanded belief in the graph.
         * @return the next belief
         */
        template <typename output = BeliefStateGraph<TBeliefState, TAction, TObservation>>
        std::shared_ptr<output> expand(const TAction &action, const TObservation &observation, bool backup = true);

        /**
         * @brief Return a 
         */
        std::string str();

        /**
         * @brief Return a the pointer on this object
         */
        std::shared_ptr<BeliefStateGraph<TBeliefState, TAction, TObservation>> getptr();

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);

        friend std::ostream &operator<<(std::ostream &os, BeliefStateGraph &belief_state_graph)
        {
            os << belief_state_graph.str();
            return os;
        }

    protected:
        /**
         * @brief Return the successor node. Given an action and an observation, this function will return the next belief.
         * 
         * @param action the action
         * @param observation the observation
         * @return the next belief state 
         */
        std::pair<TBeliefState, double> computeNextBelief(const TAction &action, const TObservation &observation);

        /**
         * @brief The dynamics of states. 
         */
        std::shared_ptr<std::vector<std::vector<DenseMatrix<typename TBeliefState::state_type, typename TBeliefState::state_type>>>> dynamics_;

        // std::shared_ptr<BinaryFunction<TAction, TObservation, std::shared_ptr<MatrixInterface<TBeliefState::state_type, TBeliefState::state_type>>>> dynamics_;

        /**
         * @brief A pointer on the bag containing all nodes.
         */
        std::shared_ptr<std::unordered_map<TBeliefState, std::shared_ptr<BeliefStateGraph>>> belief_space;

        /**
         * @brief A pointer on the bag containing all nodes.
         */
        RecursiveMap<TAction, TObservation, double> belief_proba;
    };

} // namespace sdm
#include <sdm/core/state/belief_state_graph.tpp>
