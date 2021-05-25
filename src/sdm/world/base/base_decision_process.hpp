

template <typename TState, typename TAction, typename TObservation>
class BaseDecisionProcess
{
    

    virtual number getNumAgents() = 0;

    virtual number getNumStates() = 0;
    virtual std::shared_ptr<DiscreteSpace<TState>> getStateSpace() = 0;

    virtual number getNumActions() = 0;
    virtual std::shared_ptr<DiscreteSpace<TAction>> getActionSpace() = 0;

    virtual number getNumObservations() = 0;
    virtual std::shared_ptr<DiscreteSpace<TObservation>> getObsSpace() = 0;

    virtual std::shared_ptr<StateDynamics<TState, TAction, TObservation>> getStateDynamics() = 0;
    virtual double getTransitionProbability(const TState &state, const TAction &action, const TState &next_state) = 0;

    virtual std::shared_ptr<ObsDynamics<TState, TAction, TObservation>> getObsDynamics() = 0;
    virtual std::shared_ptr<Vector<TObservation>> getObsDynamics(const TAction &action, const TState &next_state) = 0;
    virtual double getObservationProbability(const TAction &action, const TState &next_state, const TObservation &observation) = 0;

    virtual std::shared_ptr<Dynamics<TState, TAction, TObservation>> getDynamics() = 0;
    virtual std::shared_ptr<Matrix<TState, TState>> getDynamics(const TAction &action, const TObservation &observation) = 0;
    virtual double getDynamics(const TState &state, const TAction &action, const TState &next_state, const TObservation &observation) = 0;

    virtual std::shared_ptr<Reward<TState, TAction>> getReward() = 0;
    virtual std::shared_ptr<Matrix<TState>> getReward(const TAction &action) = 0;
    virtual std::shared_ptr<Matrix<TAction>> getReward(const TState &state) = 0;
    virtual std::shared_ptr<Vector<number>> getReward(const TState &state, const TAction &action) = 0;
    virtual double getReward(const TState &state, const TAction &action, const number& agent_id) = 0;

    virtual std::shared_ptr<Vector<TState>> getStartDistribution() = 0;

};