#pragma once
// #include <set>

// #include <sdm/core/state/serialized_state.hpp>
// #include <sdm/utils/linear_algebra/vector.hpp>
// #include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/value_function_new_interface.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    class HyperplanValueFunction : public ValueFunctionNewInterface
    {
    public:
        HyperplanValueFunction();

        /**
         * @brief Construct a new Hyperplan Value Function object
         * 
         * @param number : horizon 
         * @param std::shared_ptr<Initializer<TState, TAction>> : initializer 
         * @param int frequency of the pruning 
         */
        HyperplanValueFunction(number, std::shared_ptr<Initializer>, int = 10);

        /**
         * @brief Construct a new Hyerplan Value Function object
         * 
         * @param number : horizon 
         * @param TValue : initializer 
         * @param int frequency of the pruning 
         */
        HyperplanValueFunction(number = 0, double = 0., int = 10);
        ~HyperplanValueFunction();

        void initialize();
        void initialize(double, number = 0);

        /**
         * @brief Get the Value at state x.
         * 
         * @param state the state 
         * @return double 
         */
        double getValueAt(const std::shared_ptr<State> &, number = 0);

        /**
         * @brief Update the max plan representation by adding a new hyperplan
         */
        void updateValueAt(const std::shared_ptr<State> &, number = 0);

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::vector<std::shared_ptr<State>> getSupport(number);

        /**
         * @brief Get the maximum value and hyperplan at a specific state
         * 
         * @param state a specific state
         * @return the maximum value and hyperplan at a specific state (std::pair<double, std::shared_ptr<State>>) 
         */
        std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<State> &, number);

        /**
         * @brief Prune unecessary vectors
         * 
         */
        void prune(number = 0);

        /*!
         * @brief this method prunes dominated alpha-vectors, known as Lark's pruning.
         * This approach goes other all vectors until all of them have been treated. For each arbitrary vector,
         * it performs a linear programming providing a gap delta, and a frequency f. If the gap is over a certain
         * threshold epsilon, that means one can preserve the selected vector, otherwise one should discard it.
         */
        void lark_prune(number = 0);

        /*!
         * @brief this method prunes dominated points, known as bounded pruning by Trey Smith.
         * This approach stores the number of frequency states, among those already visited, that are maximal at a hyperplan.
         * And prune hyperplan with a number of maximal frequency states zero.
         */
        void bounded_prune(number = 0);

        /**
         * @brief this method prunes dominated vectors, known as Pairwise pruning.
         * 
         * @param number : timestep 
         */
        void pairwise_prune(number t);


        std::string str() const;

    protected:
        using HyperplanSet = std::vector<std::shared_ptr<State>>;

        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         */
        std::vector<HyperplanSet> representation;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<Initializer> initializer_;

        /**
         * @brief the default values, one for each decision epoch.
         */
        std::vector<double> default_values_per_horizon;

        /**
         * @brief Frequency before prunning
         * 
         */
        number freq_prune_;

        /**
         * @brief The last time the prunning took place
         * 
         */
        number last_prunning = 0;
    };

} // namespace sdm
#include <sdm/utils/value_function/hyperplan_value_function.tpp>
