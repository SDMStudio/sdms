
template <typename TState, typename TAction>
class BaseStateDynamics
{
protected:
    //! \brief transition and observation matrices
    RecursiveMap<TState, TAction, TState, double> t_model; //
    // RecursiveMap<TAction, TState<MappedVector<TObservation>> o_model;
    // RecursiveMap<TAction, <MappedMatrix<TState,TObservation>> o_model;

public:
    double getTransitionProbability(TState, TAction, TState) const;
    void setTransitionProbability(TState, TAction, TState, double);
};