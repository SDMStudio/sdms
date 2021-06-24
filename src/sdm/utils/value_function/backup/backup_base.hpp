#pragma once

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/linear_algebra/vector_interface.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/backup/backup_interface.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{

    template<typename TData>
    class BackupBase : public BackupInterface
    {
    public:

        BackupBase();
        BackupBase(const std::shared_ptr<SolvableByHSVI>& world);
        
        virtual ~BackupBase();
        /**
         * @brief Get the max value of a state
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return std::pair<double, std::shared_ptr<State>> 
         */
        virtual std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
        
        /**
         * @brief 
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return TData 
         */
        virtual TData backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
        
        /**
         * @brief Get the best action for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return std::shared_ptr<Action> : Action
         */
        virtual std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValueAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);
    protected : 
        std::shared_ptr<SolvableByHSVI> world_;
    };

} // namespace sdm
#include <sdm/utils/value_function/backup/backup_base.tpp>
