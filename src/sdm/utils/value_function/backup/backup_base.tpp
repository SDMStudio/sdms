namespace sdm
{
    template<typename TData>
    BackupBase<TData>::BackupBase(){}

    template<typename TData>
    BackupBase<TData>::BackupBase(const std::shared_ptr<SolvableByHSVI>& world): world_(world){}

    template<typename TData>
    BackupBase<TData>::~BackupBase(){}
}