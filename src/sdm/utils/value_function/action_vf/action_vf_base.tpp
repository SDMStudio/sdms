namespace sdm
{
    template<typename TData>
    ActionVFBase<TData>::ActionVFBase(){}

    template<typename TData>
    ActionVFBase<TData>::ActionVFBase(const std::shared_ptr<SolvableByHSVI>& world): world_(world){}

    template<typename TData>
    ActionVFBase<TData>::~ActionVFBase(){}
}