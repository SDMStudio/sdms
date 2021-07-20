#pragma once
// #define LOGTIME

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/backup/backup_interface.hpp>

#include <bits/stdc++.h>
#include <sdm/config.hpp>
namespace sdm
{
    class HyperplanValueFunction : public ValueFunction
    {
    public:
        static double PRECISION;

        HyperplanValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf, int freq_prunning = -1,TypeOfMaxPlanPrunning type_of_maxplan_prunning = TypeOfMaxPlanPrunning::PAIRWISE);

        HyperplanValueFunction(number horizon, double default_value = 0., const std::shared_ptr<BackupInterfaceForValueFunction> &backup = nullptr, const std::shared_ptr<ActionVFInterface> &action_vf = nullptr, int freq_prunning = -1, TypeOfMaxPlanPrunning type_of_maxplan_prunning = TypeOfMaxPlanPrunning::PAIRWISE);

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
        
        size_t getSize(number t) const;

        std::string str() const;

        /**
         * @brief Get the Default Value at time step t
         * 
         * @param number : timestep
         * 
         * @return double 
         */
        double getDefaultValue(number);

        bool exist(const std::shared_ptr<BeliefInterface>&,number t, double precision = PRECISION);

        Pair<std::shared_ptr<State>, double> evaluate(const std::shared_ptr<State> &state, number t);

        void createDefault(const std::shared_ptr<State> &state, number t);

        void do_pruning(number t);

    protected:
        using HyperplanSet = std::vector<std::shared_ptr<State>>;
        // using HyperplanSet = std::unordered_set<std::shared_ptr<State>, Hash, KeyEqual>;

        // std::vector<std::shared_ptr<VectorInterface<std::shared_ptr<State>,double>>> getAllVectorInterface(number t);

        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         */
        std::vector<HyperplanSet> representation;
        /**
         * @brief the default values, one for each decision epoch.
         */
        std::vector<double> default_values_per_horizon;

        /**
         * @brief Frequency before prunning
         * 
         */
        number freq_pruning_;

        /**
         * @brief The last time the prunning took place
         * 
         */
        number last_prunning = 0;

        TypeOfMaxPlanPrunning type_of_maxplan_prunning_;
        std::vector<std::unordered_set<std::shared_ptr<State>>> all_state_updated_so_far;

    };

} // namespace sdm
