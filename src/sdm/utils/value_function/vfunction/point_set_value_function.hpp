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
    class BasePointSetValueFunction : public BaseTabularValueFunction<Hash, KeyEqual>,
                                      public PrunableStructure
    {
    public:
        using Container = typename BaseTabularValueFunction<Hash, KeyEqual>::Container;

        BasePointSetValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                  const std::shared_ptr<Initializer> &initializer,
                                  const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                  const std::shared_ptr<TabularUpdateOperator> &update_operator,
                                  int freq_prunning = -1,
                                  TypeOfSawtoothPrunning type_of_sawtooth_prunning = TypeOfSawtoothPrunning::NONE);

        /**
         * @brief Evaluate the value at a state.
         *
         * @param state the state where we want to evaluate the function
         * @return the value
         */
        double getValueAt(const std::shared_ptr<State> &state, number t);

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

        std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, BasePointSetValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    protected:
        /**
         * @brief Type of pruning.
         *
         */
        TypeOfSawtoothPrunning type_of_sawtooth_prunning_;

        /**
         * @brief Is set to true if the structure use a linear program
         * to select the greedy action (should be avoid). 
         * 
         */
        bool is_sawtooth_lp = false;

        /**
         * @brief Point-wise pruning.
         *
         */
        void prune(number t = 0);

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

        Pair<std::unordered_map<std::shared_ptr<State>, std::vector<std::shared_ptr<State>>>, std::map<int, std::vector<std::shared_ptr<State>>>> iterative_pruning(number t);

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

    using PointSetValueFunction = BasePointSetValueFunction<std::hash<std::shared_ptr<State>>, std::equal_to<std::shared_ptr<State>>>;
    using PointSetValueFunction2 = BasePointSetValueFunction<sdm::hash_from_ptr<State>, sdm::equal_from_ptr<State>>;

} // namespace sdm

#include <sdm/utils/value_function/vfunction/point_set_value_function.tpp>
