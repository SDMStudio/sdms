namespace sdm
{
    template <typename TState, typename TAction, typename TObservation>
    BaseObsDynamics<TState,TAction,TObservation>::BaseObsDynamics()
    {
    }

    // template <typename TState, typename TAction, typename TObservation>
    // BaseObsDynamics::BaseObsDynamics(BaseObsDynamics &copy) : o_model(copy.getObservationProbabilities()), dynamics(copy.getDynamics())
    // {
    // }

    template <typename TState, typename TAction, typename TObservation>
    double BaseObsDynamics<TState,TAction,TObservation>::getObservationProbability(TAction u, TObservation z, TState x) const
    {
        return this->o_model(u,x,z);
    }

    template <typename TState, typename TAction, typename TObservation>
    void BaseObsDynamics<TState,TAction,TObservation>::setObservationProbability(TAction u, TObservation z, TState x, double prob)
    {
        this->o_model.recursive_emplace(u,x,z,prob);
    }

    template <typename TState, typename TAction, typename TObservation>
    double BaseObsDynamics<TState,TAction,TObservation>::getDynamics(TState x , TAction u , TObservation z , TState y) const
    {
        return this->dynamics(x,u,z,y);
    }

    template <typename TState, typename TAction, typename TObservation>
    void BaseObsDynamics<TState,TAction,TObservation>::setDynamics(TState x , TAction u , TObservation z , TState y, double prob)
    {
        this->dynamics.recursive_emplace(x,u,z,y,prob);
    }

}