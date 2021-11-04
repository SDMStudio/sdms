#pragma once

#include <bits/stdc++.h>
#include <sdm/config.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>
#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>

namespace sdm
{
    class PWLCValueFunction : public ValueFunction, public PWLCValueFunctionInterface
    {
    public:
        static double PRECISION;

        PWLCValueFunction(const std::shared_ptr<SolvableByDP> &world,
                          const std::shared_ptr<Initializer> &initializer,
                          const std::shared_ptr<ActionSelectionInterface> &action_selection,
                          const std::shared_ptr<PWLCUpdateOperator> &update_operator,
                          int freq_prunning = -1,
                          MaxplanPruning::Type type_of_maxplan_prunning = MaxplanPruning::Type::PAIRWISE);
                          
        PWLCValueFunction(const PWLCValueFunction &copy);

        /**
         * @brief Initialize the value function by using initializer.
         */
        void initialize();

        /**
         * @brief Set all values of the vector to a default value.
         *
         * @param default_value the default value
         */
        void initialize(double, number = 0);

        /**
         * @brief Get the Value at state x.
         *
         * @param state the state
         * @return double
         */
        double getValueAt(const std::shared_ptr<State> &, number = 0);

        /**
         * @brief Add a hyperplane in the hyperplane set.
         *
         * This fonction will addd the hyperplane called new_hyperplane in the
         * set of plans at a specific time step. 
         * 
         * @param state the state
         * @param new_hyperplane the new hyperplane
         * @param t the timestep
         */
        void addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<State> &new_hyperplane, number t);

        /**
         * @brief Get the set of hyperplanes at a time step
         *
         * @return the list of hyperplanes
         */
        std::vector<std::shared_ptr<State>> getHyperplanesAt(const std::shared_ptr<State>& state, number t);

        /**
         * @brief Get the best hyperplane resulting to the higher value when evaluated at a given state.
         * 
         * @param state the state
         * @param t the time step
         * @return the max hyperplane 
         */
        std::shared_ptr<State> getHyperplaneAt(const std::shared_ptr<State> &state, number t);

        std::vector<std::shared_ptr<State>> getSupport(number t);

        double getBeta(const std::shared_ptr<State> &alpha, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Get the Default Value at time step t
         *
         * @param number : timestep
         *
         * @return double
         */
        double getDefaultValue(number);

        /**
         * @brief Get the maximum value and corresponding hyperplane at a specific state
         *
         * @param state a specific state
         * @param t the time step
         * @return the maximum value and hyperplane at a specific state
         */
        Pair<std::shared_ptr<State>, double> evaluate(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Get the size of the value function at timestep t
         */
        size_t getSize(number t) const;

        /**
         * @brief Get a string representation of this class.
         */
        std::string str() const;

        /**
         * @brief Copy the value function and return a reference to the copied object.
         * 
         * @return the address of the value function copied
         */
        std::shared_ptr<ValueFunctionInterface> copy();


    protected:
        using HyperplanSet = std::vector<std::shared_ptr<State>>;
        // using HyperplanSet = std::unordered_set<std::shared_ptr<State>, Hash, KeyEqual>;

        /**
         * @brief The value function represention.
         * 
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         */
        std::vector<HyperplanSet> representation;

        /**
         * @brief the default values, one for each decision epoch.
         */
        std::vector<double> default_values_per_horizon;

        /**
         * @brief The type of pruning used.
         */
        MaxplanPruning::Type type_of_maxplan_prunning_;

        std::vector<std::unordered_set<std::shared_ptr<State>>> all_state_updated_so_far;

        /**
         * @brief Prune dominated hyperplanes of the value function.
         * 
         * @param t the time step
         */
        void prune(number t);

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
         * And prune hyperplane with a number of maximal frequency states zero.
         */
        void bounded_prune(number = 0);

        /**
         * @brief this method prunes dominated vectors, known as Pairwise pruning.
         *
         * @param number : timestep
         */
        void pairwise_prune(number t);

        double getNextAlphaValue(const std::shared_ptr<State> &alpha, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation);
        double getNextAlphaValueBelief(const std::shared_ptr<State> &alpha, const std::shared_ptr<State> &, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &);
        double getNextAlphaValueOccupancy(const std::shared_ptr<State> &alpha, const std::shared_ptr<State> &, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation);
    };

} // namespace sdm
