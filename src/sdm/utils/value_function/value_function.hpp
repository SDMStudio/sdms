#pragma once
// #define LOGTIME

#include <sdm/core/function.hpp>
#include <sdm/utils/value_function/base_value_function.hpp>
#include <sdm/utils/value_function/action_vf/action_vf_interface.hpp>
#include <sdm/utils/value_function/backup/backup_interface.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    class Initializer;

    /**
     * @class ValueFunction
     * @brief This class is the abstract class of value function. All value function must derived this class.
     * 
     * @tparam std::shared_ptr<State> Type of the state.
     * @tparam std::shared_ptr<Action> Type of the action.
     * @tparam double Type of the value.
     */
    class ValueFunction
        : public BaseValueFunction,
          public BinaryFunction<std::shared_ptr<State>, number, double>

    {
    public:
        ValueFunction() {}

        /**
         * @brief Construct a new Incremental Value Function object
         * 
         * @param backup 
         * @param default_value 
         */
        ValueFunction(number horizon = 0, const std::shared_ptr<Initializer> &intializer = nullptr, const std::shared_ptr<BackupInterfaceForValueFunction> &backup = nullptr, const std::shared_ptr<ActionVFInterface> &action = nullptr);

        /**
         * @brief Destroy the value function
         * 
         */
        virtual ~ValueFunction() {}

        /**
         * @brief Initialize the value function 
         */
        virtual void initialize() = 0;

        /**
         * @brief Initialize the value function with a default value
         */
        virtual void initialize(double v, number t = 0) = 0;

        /**
         * @brief Set a function as a interactive way to get initial values for state that are not already initialized. 
         * 
         * @param init_function the function that enables to get initial values 
         */
        void initialize(const std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> &init_function);

        /**
         * @brief Get the value at a given state
         */
        virtual double getValueAt(const std::shared_ptr<State> &state, number t = 0) = 0;

        /**
         * @brief Update the value at a given state
         */
        virtual void updateValueAt(const std::shared_ptr<State> &state, number t = 0) = 0;

        /**
         * @brief Return the possible indexes of the value function
         * 
         * @return std::string 
         */
        virtual std::vector<std::shared_ptr<State>> getSupport(number t) = 0;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const = 0;

        /** @brief Get the size of the value function at timestep t */
        virtual size_t getSize(number t) const = 0;

        /** @brief Get the total size of the value function. */
        size_t getSize() const;

        /**
         * @brief Get shared pointer on the current QValueFunction
         */
        std::shared_ptr<ValueFunction> getptr();

        double operator()(const std::shared_ptr<State> &state, const number &t = 0);

        std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> getInitFunction();

        /**
         * @brief Evaluate the element given
         * 
         * @param state : ELement to evaluate
         * @param t 
         * @return Pair<std::shared_ptr<State>, double> 
         */
        virtual Pair<std::shared_ptr<State>, double> evaluate(const std::shared_ptr<State> &state, number t) = 0;

        /**
         * @brief Do the backup operator
         * 
         * @param state : ELement to evaluate
         * @param t 
         * @return Pair<std::shared_ptr<State>, double> 
         */
        template <typename TData>
        TData backup(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
        {
            #ifdef LOGTIME
                this->StartTime();
            #endif

            auto backup = std::static_pointer_cast<BackupInterface<TData>>(this->backup_)->backup(this->getptr(),state,action,t);

            #ifdef LOGTIME 
                this->updateTime("backup");
            #endif

            return backup;
        }

        /**
         * @brief Select the best Action for a precise state and a time step
         * 
         * @param const std::shared_ptr<State> & : current state 
         * @param number : time step 
         * @return std::shared_ptr<Action> 
         */
        std::shared_ptr<Action> getBestAction(const std::shared_ptr<State> &state, number t);

        #ifdef LOGTIME
            double time_start;
            double total_time_update_backup =0;
            double total_time_update_best_action =0;
            double total_time_evaluate =0;
            double total_time_exist =0;

            void StartTime();
            void updateTime(std::string information);
            void printTime();
        #endif

        virtual void do_pruning(number t) =0;

    protected:
        /**
         * @brief Initialization function. If defined, algorithms on value functions will get inital values using this function.
         */
        std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> init_function_ = nullptr;

        /**
         * @brief The evaluate operator.
         */
        // std::shared_ptr<EvaluateVFInterface> evaluate_;

        /**
         * @brief The backup operator.
         */
        std::shared_ptr<BackupInterfaceForValueFunction> backup_;

        /**
         * @brief The action operator.
         */
        std::shared_ptr<ActionVFInterface> action_;

        /**
         * @brief The initializer to use for this value function. 
         */
        std::shared_ptr<Initializer> initializer_;
    };
} // namespace sdm