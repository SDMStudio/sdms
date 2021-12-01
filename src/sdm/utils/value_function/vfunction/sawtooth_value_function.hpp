#pragma once

#include <sdm/utils/value_function/prunable_structure.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief The value function that is represented using a set of points 
     * ([See Tutorials](https://sdmstudio.github.io/tutorials/value_function/v/#point-set-value)).
     * 
     * Similar to the tabular rerpresentation, the point set representation keep points ( i.e. state/value pairs ) 
     * but add a way to generalize over new states using an interpolation over existing points in the structure.
     * 
     * @tparam Hash the type of hash function
     * @tparam KeyEqual the type of equal function
     */
    template <class Hash = std::hash<std::shared_ptr<State>>, class KeyEqual = std::equal_to<std::shared_ptr<State>>>
    class BaseSawtoothValueFunction : public BaseTabularValueFunction<Hash, KeyEqual>,
                                      public PrunableStructure
    {
    public:
        using Container = typename BaseTabularValueFunction<Hash, KeyEqual>::Container;

        BaseSawtoothValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                  const std::shared_ptr<Initializer> &initializer,
                                  const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                  const std::shared_ptr<TabularUpdateOperator> &update_operator,
                                  int freq_prunning = -1,
                                  SawtoothPruning::Type type_of_sawtooth_prunning = SawtoothPruning::PAIRWISE);

        BaseSawtoothValueFunction(const BaseSawtoothValueFunction &copy);

        /**
         * @brief Evaluate the value at a state.
         *
         * @param state the state where we want to evaluate the function
         * @return the value
         */
        double getValueAt(const std::shared_ptr<State> &state, number t);

        void setValueAt(const std::shared_ptr<State> &state, double new_value, number t);

        /**
         * @brief Evaluate the element given
         *
         * @param state : ELement to evaluate
         * @param t
         * @return Pair<std::shared_ptr<State>, double>
         */
        Pair<std::shared_ptr<State>, double> evaluate(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Get the relaxed value at a given state
         * 
         * Usually, the relaxation correspond to the underlying MDP value or belief MDP value. 
         * However, we could use other types of relaxations such as the hierarchical value, etc.
         * 
         * @param state the state the evaluate
         * @param t the time step
         * @return the relaxed value 
         */
        double getRelaxedValueAt(const std::shared_ptr<State> &state, number t);

        double getSawtoothValueAt(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<JointHistoryInterface> joint_history,
                                  const std::shared_ptr<Action> &action, const std::shared_ptr<OccupancyStateInterface> &next_occupancy_state,
                                  const std::shared_ptr<JointHistoryInterface> &next_joint_history,
                                  const std::shared_ptr<Observation> &next_observation, number t, bool display = false);

        /**
         * @brief Copy the value function and return a reference to the copied object.
         * 
         * @return the address of the value function copied
         */
        std::shared_ptr<ValueFunctionInterface> copy();

        /**
         * @brief Get a string representation of this class.
         */
        std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, BaseSawtoothValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    protected:
        /**
         * @brief Type of pruning.
         *
         */
        SawtoothPruning::Type type_of_sawtooth_prunning_;

        /**
         * @brief A data structure to store values of the relaxations.
         */
        std::vector<Container> relaxation;

        /**
         * @brief A data structure to store values of the relaxations.
         */
        RecursiveMap<std::shared_ptr<State>, std::shared_ptr<State>, double> ratios;

        /**
         * @brief Point-wise pruning.
         *
         */
        void prune(number t = 0);

        /**
         * @brief This method prunes dominated points, known as Pairwise pruning.
         * 
         * @param t the timestep 
         * @param epsilon the epsilon used to check dominance between states 
         */
        void pairwise_prune(number t, double epsilon = 0.);

        /**
         * @brief Compute the sawtooth ratio for the evaluate function
         *
         * @param s the state
         * @param s_k the k-th state stored in the map
         * @return the ratio 
         */
        double computeRatio(const std::shared_ptr<State> &s, const std::shared_ptr<State> &s_k);

        /**
         * @brief Ratio specialized for the Occupancy case (used for the evaluate function)
         *
         * @param s the new occupancy state
         * @param s_k the k-th occupancy state stored in the map
         * @return the minimum ratio between s(x,o)/s^k(x,o)  
         */
        double ratioOccupancy(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<OccupancyStateInterface> &s_k);

        /**
         * @brief Ratio specialized for the belief case (used for the evaluate function)
         *
         * @param b the new belief
         * @param b_k the k-th belief stored in the map
         * @return the minimum ratio between b(x)/b^k(x)  
         */
        double ratioBelief(const std::shared_ptr<BeliefInterface> &b, const std::shared_ptr<BeliefInterface> &b_k);

    public:
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &archive, const unsigned int)
        {
            using boost::serialization::make_nvp;

            archive &make_nvp("horizon", this->horizon_);
            archive &make_nvp("representation", this->representation);
        }
    };

    /** @brief Point set value function using address comparison */
    using SawtoothValueFunction = BaseSawtoothValueFunction<std::hash<std::shared_ptr<State>>, std::equal_to<std::shared_ptr<State>>>;

    /** @brief Point set value function using state content comparison */
    using SawtoothValueFunction2 = BaseSawtoothValueFunction<sdm::hash_from_ptr<State>, sdm::equal_from_ptr<State>>;

} // namespace sdm

#include <sdm/utils/value_function/vfunction/sawtooth_value_function.tpp>
