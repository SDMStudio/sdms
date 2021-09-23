namespace sdm
{
    template <class TCondition, class TState>
    QValueFunctionConditioning<TCondition, TState>::QValueFunctionConditioning()
    {
    }

    template <class TCondition, class TState>
    QValueFunctionConditioning<TCondition, TState>::QValueFunctionConditioning(number horizon) : QValueFunction<Pair<TCondition,TState>>(horizon)
    {
    }

    template <class TCondition, class TState>
    std::shared_ptr<QValueFunctionConditioning<TCondition, TState>> QValueFunctionConditioning<TCondition, TState>::getptr()
    {
        return std::static_pointer_cast<QValueFunctionConditioning<TCondition, TState>>(this->shared_from_this());
    }

    template <class TCondition, class TState>
    double QValueFunctionConditioning<TCondition, TState>::getQValueAt(const Pair<TCondition, TState> &input, const std::shared_ptr<Action> &action, number t)
    {
        return this->getQValueAt(input.first,input.second,action,t);
    }

    template <class TCondition, class TState>
    void QValueFunctionConditioning<TCondition, TState>::updateQValueAt(const Pair<TCondition, TState> &input, const std::shared_ptr<Action> &action, number t)
    {
        this->updateQValueAt(input.first,input.second,action,t);
    }

    template <class TCondition, class TState>
    void QValueFunctionConditioning<TCondition, TState>::updateQValueAt(const Pair<TCondition, TState> &input, const std::shared_ptr<Action> &action, number t, double target)
    {
        this->updateQValueAt(input.first,input.second,action,t,target);
    }



} // namespace sdm