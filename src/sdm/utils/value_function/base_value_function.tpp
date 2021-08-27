namespace sdm
{
    template <class TInput>
    BaseValueFunction<TInput>::BaseValueFunction() {}

    template <class TInput>
    BaseValueFunction<TInput>::BaseValueFunction(number horizon) : horizon_(horizon) {}
    
    template <class TInput>
    BaseValueFunction<TInput>::BaseValueFunction(const BaseValueFunction& copy) : horizon_(copy.horizon_) {}

    template <class TInput>
    number BaseValueFunction<TInput>::getHorizon() const
    {
        return this->horizon_;
    }

    template <class TInput>
    bool BaseValueFunction<TInput>::isFiniteHorizon() const
    {
        return (this->horizon_ > 0);
    }

    template <class TInput>
    bool BaseValueFunction<TInput>::isInfiniteHorizon() const
    {
        return !(this->isFiniteHorizon());
    }

    template <class TInput>
    std::shared_ptr<BaseValueFunction<TInput>> BaseValueFunction<TInput>::getptr()
    {
        return this->shared_from_this();
    }
}