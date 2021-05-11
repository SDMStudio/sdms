
#pragma once
#include <set>

#include <sdm/core/state/serialized_state.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief 
     * 
     * @tparam TState type of hyperplan representation. Must implement sdm::VectorImpl interface.
     * @tparam TValue value type (default : double)
     */
    template <typename TState, typename TAction, typename TValue = double>
    class SawtoothValueFunction : public MappedValueFunction<TState, TAction, TValue>
    {
    protected:
        TState_t ctype = COMPRESSED;

        TState_t getTStateType();
        void setTStateType(const TState_t &);

    public:
        SawtoothValueFunction();
        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, std::shared_ptr<Initializer<TState, TAction>> initializer);
        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon = 0, TValue default_value = 0.);

        /**
         * @brief Evaluate the value at a state.
         * 
         * @param state the state where we want to evaluate the function
         * @return the value
         */
        TValue getValueAt(const TState &state, number t = 0);

        void updateValueAt(const TState &state, number t);

        std::pair<TValue, TState> getMaxAt(const TState &state, number t);

        /**
         * @brief Point-wise pruning
         * 
         */
        void prune(number t = 0);

        bool is_dominated(const TState &ostate, double value, number t);
    };

    template <class TAction, class TValue>
    class SawtoothValueFunction<number, TAction, TValue> : public MappedValueFunction<number, TAction, TValue>
    {
    public:
        SawtoothValueFunction()
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
        }
        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<number, TAction>>, int = 0, TValue = 0.)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
        }

        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<number, TAction>> problem, int horizon, std::shared_ptr<Initializer<number, TAction>> initializer) : MappedValueFunction<number, TAction, TValue>(problem, horizon, initializer)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
        }
        void initialize(TValue, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
        }

        TValue getValueAt(const number &, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
        }

        void updateValueAt(const number &, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
        }

        std::vector<number> getSupport(number)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
        }

        std::string str()
        {
            return "MaxPlanVF";
        }
    };

    template <class TAction, class TValue>
    class SawtoothValueFunction<SerializedState, TAction, TValue> : public MappedValueFunction<SerializedState, TAction, TValue>
    {
    public:
        SawtoothValueFunction()
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
        }
        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<SerializedState, TAction>>, int = 0, TValue = 0.)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
        }

        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<SerializedState, TAction>> problem, int horizon, std::shared_ptr<Initializer<SerializedState, TAction>> initializer) : MappedValueFunction<SerializedState, TAction, TValue>(problem, horizon, initializer)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
        }
        void initialize(TValue, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
        }

        TValue getValueAt(const SerializedState &, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
        }

        void updateValueAt(const SerializedState &, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
        }

        std::vector<SerializedState> getSupport(number)
        {
            throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
        }

        std::string str()
        {
            return "MaxPlanVF";
        }
    };
}
#include <sdm/utils/value_function/sawtooth_vf.tpp>
