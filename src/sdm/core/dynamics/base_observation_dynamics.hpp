
template <typename TState, typename TAction, typename TObservation>
class BaseObsDynamics {
    

    protected:
        //! \brief transition and observation matrices
        RecursiveMap<TAction, TState, TObservation, double> o_model; // 
        // RecursiveMap<TAction, TState<MappedVector<TObservation>> o_model;
        // RecursiveMap<TAction, <MappedMatrix<TState,TObservation>> o_model;

        //! \brief dynamics model of the probabilities of state-observation pairs given state-action pairs.
        RecursiveMap<TState, TAction, TObservation, TState, double> dynamics; // 
        // std::vector<std::vector<Matrix>> dynamics;

    public:
        double getObservationProbability(TAction, TObservation, TState) const;
        void setObservationProbability(TAction, TObservation, TState, double);


        double getDynamics(TState, TAction, TObservation, TState) const;
        void setDynamics(TState, TAction, TObservation, TState, double);
};