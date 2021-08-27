#include <iterator>
#include <sdm/utils/value_function/initializer/initializer.hpp>

namespace sdm
{
    template <class TCondition, class TState>
    TabularQValueFunctionConditioning<TCondition, TState>::TabularQValueFunctionConditioning(number horizon, double learning_rate, std::shared_ptr<QInitializer<TInput>> initializer, bool active_learning)
        : TabularQValueFunction<Pair<TCondition,TState>>(horizon,learning_rate,initializer,active_learning) , QValueFunctionConditioning<TCondition,TState>(horizon)
    {
    }

    template <class TCondition, class TState>
    TabularQValueFunctionConditioning<TCondition, TState>::TabularQValueFunctionConditioning(number horizon, double learning_rate, double default_value, bool active_learning) : TabularQValueFunctionConditioning<TCondition, TState>(horizon, learning_rate, std::make_shared<ValueInitializer<TInput>>(default_value),active_learning)
    {
    }

    template <class TCondition, class TState>
    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> TabularQValueFunctionConditioning<TCondition, TState>::getQValuesAt(const TInput &state, number t)
    {
        using v_type = typename MappedMatrix<TInput, std::shared_ptr<Action>, double>::value_type::second_type;
        auto h = this->isInfiniteHorizon() ? 0 : t;
        return std::make_shared<v_type>(this->representation[h].at(state));
    }

    template <class TCondition, class TState>
    double TabularQValueFunctionConditioning<TCondition, TState>::getQValueAt(const TInput &input, const std::shared_ptr<Action> &action, number t)
    {
        QValueFunctionConditioning<TCondition,TState>::getQValueAt(input,action,t);
    }

    template <class TCondition, class TState>
    double TabularQValueFunctionConditioning<TCondition, TState>::getQValueAt(const TCondition &condition, const TState& state, const std::shared_ptr<Action> &action, number t)
    {
        TabularQValueFunction<Pair<TCondition,TState>>::getQValueAt(std::make_pair(condition,state),action,t);
    }

    template <class TCondition, class TState>
    void TabularQValueFunctionConditioning<TCondition, TState>::updateQValueAt(const TInput &input, const std::shared_ptr<Action> &action, number t, double delta)
    {
        QValueFunctionConditioning<TCondition,TState>::updateQValueAt(input,action,t);
    }

    template <class TCondition, class TState>
    void TabularQValueFunctionConditioning<TCondition, TState>::updateQValueAt(const TCondition &condition, const TState& state, const std::shared_ptr<Action> &action, number t, double delta)
    {
        TabularQValueFunction<Pair<TCondition,TState>>::updateQValueAt(std::make_pair(condition,state),action,t);
    }

    template <class TCondition, class TState>
    void TabularQValueFunctionConditioning<TCondition, TState>::updateQValueAt(const TInput &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }


} // namespace sdm