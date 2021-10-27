#pragma once

#include <bits/stdc++.h>
#include <sdm/config.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>
#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>

namespace sdm
{
    class PWLCValueFunction : public ValueFunction, virtual public PWLCValueFunctionInterface
    {
    public:
        static double PRECISION;

        PWLCValueFunction(
            const std::shared_ptr<SolvableByDP> &world,
            const std::shared_ptr<Initializer> &initializer,
            const std::shared_ptr<ActionSelectionInterface> &action_selection,
            const std::shared_ptr<PWLCUpdateOperator> &update_operator,
            int freq_prunning = -1,
            TypeOfMaxPlanPrunning type_of_maxplan_prunning = TypeOfMaxPlanPrunning::PAIRWISE);

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
         * @brief Add a hyperplane in the hyperplan set.
         *
         * This fonction will addd the hyperplan called new_hyperplan in the
         * set of plans at a specific time step. 
         * 
         * @param state the state
         * @param new_hyperplane the new hyperplane
         * @param t the timestep
         */
        void addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<State> &new_hyperplan, number t);

        /**
         * @brief Get the set of hyperplanes at a time step
         *
         * @return the list of hyperplanes
         */
        std::vector<std::shared_ptr<State>> getHyperplanesAt(number t);
        
        std::vector<std::shared_ptr<State>> getSupport(number t);

        void getBeta(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Get the Default Value at time step t
         *
         * @param number : timestep
         *
         * @return double
         */
        double getDefaultValue(number);

        /**
         * @brief Evaluate the element given
         *
         * @param state : ELement to evaluate
         * @param t
         * @return Pair<std::shared_ptr<State>, double>
         */
        Pair<std::shared_ptr<State>, double> evaluate(const std::shared_ptr<State> &state, number t);

        size_t getSize(number t) const;

        std::string str() const;

    protected:
        using HyperplanSet = std::vector<std::shared_ptr<State>>;
        // using HyperplanSet = std::unordered_set<std::shared_ptr<State>, Hash, KeyEqual>;

        /**
         * @brief The value function represention.
         * 
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         * 
         */
        std::vector<HyperplanSet> representation;
        /**
         * @brief the default values, one for each decision epoch.
         */
        std::vector<double> default_values_per_horizon;

        /**
         * @brief The type of pruning used.
         * 
         */
        TypeOfMaxPlanPrunning type_of_maxplan_prunning_;

        std::vector<std::unordered_set<std::shared_ptr<State>>> all_state_updated_so_far;

        /**
         * @brief Prune dominated hyperplanes of the value function.
         * 
         * @param t the time step
         */
        void prune(number t);

        /**
         * @brief Create a Default object
         *
         * @param state
         * @param t
         */
        void createDefault(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Determine if the state is already stocked.
         *
         * @param t
         * @param precision
         */
        bool exist(const std::shared_ptr<BeliefInterface> &, number t, double precision = PRECISION);

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

        /**
         * @brief Get the maximum value and hyperplan at a specific state
         *
         * @param state a specific state
         * @return the maximum value and hyperplan at a specific state (std::pair<double, std::shared_ptr<State>>)
         */
        std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<State> &, number);
    };

} // namespace sdm
