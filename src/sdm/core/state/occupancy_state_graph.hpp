#pragma once

#include <sdm/core/state/belief_state_graph.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
namespace sdm
{

    class POMDPInterface;

    /**
     * @brief A graph that keep all beliefs
     * 
     */
    class OccupancyStateGraph : public BeliefStateGraph, public OccupancyStateInterface
    {
    public:
        using TransitionFunction = Pair<std::shared_ptr<OccupancyStateInterface>, double>(const std::shared_ptr<POMDPInterface> &,
                                                                                  const std::shared_ptr<OccupancyStateInterface> &,
                                                                                  const std::shared_ptr<Action> &,
                                                                                  const std::shared_ptr<Observation> &,
                                                                                  number);

        OccupancyStateGraph();
        OccupancyStateGraph(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba);
        OccupancyStateGraph(const Belief &data);

        /**
         * @brief Construct a new belief 
         * 
         * @param predecessor 
         * @param belief 
         */
        OccupancyStateGraph(const std::shared_ptr<BeliefStateGraph> &predecessor, const Belief &belief);

        void initialize();

        /**
         * @brief Get the node associated to a given belief.
         * 
         * @param belief a specific belief
         * @return the address of the node 
         */
        std::shared_ptr<BeliefStateGraph> getNode(const Belief &belief);

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
        std::shared_ptr<BeliefStateGraph> next(TransitionFunction transition_function, const std::shared_ptr<POMDPInterface> &pomdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t, bool backup = true);

        /**
         * @brief Return a 
         */
        std::string str() const;

        /**
         * @brief Return a the pointer on this object
         */
        std::shared_ptr<BeliefStateGraph> getptr();

        std::vector<std::shared_ptr<State>> getStates() const;
        double getProbability(const std::shared_ptr<State> &state) const;
        void setProbability(const std::shared_ptr<State> &state, double proba);
        void addProbability(const std::shared_ptr<State> &, double proba);
        bool operator==(const std::shared_ptr<BeliefInterface> &other) const;
        double operator^(const std::shared_ptr<BeliefInterface> &other) const;
        double norm_1() const;
        size_t size() const;

        std::shared_ptr<VectorInterface<std::shared_ptr<State>, double>> getVectorInferface();

        void setDefaultValue(double);
        double getDefaultValue() const;

        TypeState getTypeState() const;

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);

        friend std::ostream &operator<<(std::ostream &os, BeliefStateGraph &belief_state_graph)
        {
            os << belief_state_graph.str();
            return os;
        }

    protected:
        /**
         * @brief A pointer on the bag containing all nodes.
         */
        std::shared_ptr<std::unordered_map<Belief, std::shared_ptr<BeliefStateGraph>>> belief_space;

        /**
         * @brief A pointer on the bag containing all nodes.
         */
        RecursiveMap<std::shared_ptr<Action>, std::shared_ptr<Observation>, double> belief_proba;
    };

} // namespace sdm
