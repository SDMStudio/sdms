namespace sdm
{
    template <typename TObservation>
    BaseObservation<TObservation>::BaseObservation() {}

    template <typename TObservation>
    BaseObservation<TObservation>::BaseObservation(const TObservation &item) : observation_(item) {}

    template <typename TObservation>
    BaseObservation<TObservation>::~BaseObservation() {}

    template <typename TObservation>
    TObservation BaseObservation<TObservation>::getObservation() const
    {
        return this->observation_;
    }

    template <typename TObservation>
    void BaseObservation<TObservation>::setObservation(const TObservation &observation)
    {
        this->observation_ = observation;
    }

    template <typename TObservation>
    std::string BaseObservation<TObservation>::str() const
    {
        std::ostringstream res;
        res << "Observation(" << this->observation_ << ")";
        return res.str();
    }

} // namespace sdm
