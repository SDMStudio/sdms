namespace sdm
{

    template <class TInput>
    QValueFunction<TInput>::QValueFunction()
    {
    }

    template <class TInput>
    QValueFunction<TInput>::QValueFunction(number horizon) : BaseValueFunction<Pair<TInput,std::shared_ptr<Action>>>(horizon)
    {
    }

    template <class TInput>
    std::shared_ptr<QValueFunction<TInput>> QValueFunction<TInput>::getptr()
    {
        return std::static_pointer_cast<QValueFunction<TInput>>(this->shared_from_this());
    }

    template <class TInput>
    double QValueFunction<TInput>::getValueAt(const Pair<TInput,std::shared_ptr<Action>> &input, number t)
    {
        return this->getQValueAt(input.first,input.second,t);
    }

    template <class TInput>
    void QValueFunction<TInput>::updateValueAt(const Pair<TInput,std::shared_ptr<Action>> &input, number t)
    {
        this->updateQValueAt(input.first,input.second,t);
    }

    template <class TInput>
    std::shared_ptr<Action> QValueFunction<TInput>::getBestAction(const Pair<TInput,std::shared_ptr<Action>> &, number )
    {
        //Pas de BestAction dans ton code Baris ? 
        throw sdm::exception::NotImplementedException();
    }


} // namespace sdm