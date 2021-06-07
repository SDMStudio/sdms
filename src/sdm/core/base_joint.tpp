namespace sdm
{
    template <typename T>
    BaseJoint<T>::BaseJoint() : std::vector<T>() {}

    template <typename T>
    BaseJoint<T>::BaseJoint(const std::vector<T> &joint_item) : std::vector<T>(joint_item) {}

    template <typename T>
    BaseJoint<T>::BaseJoint(const std::vector<number> &, const std::vector<T> &joint_item) : std::vector<T>(joint_item) {}

    template <typename T>
    BaseJoint<T>::BaseJoint(std::initializer_list<T> list_values) : std::vector<T>(list_values) {}

    // template <typename T>
    // BaseJoint<T>::~BaseJoint() {}

    template <typename T>
    const T &BaseJoint<T>::get(const number &index) const
    {
        return this->at(index);
    }

    template <typename T>
    number BaseJoint<T>::getNumAgents() const
    {
        return this->size();
    }

    template <typename T>
    T BaseJoint<T>::operator()(const number &i)
    {
        return (*this)[i];
    }

    template <typename T>
    std::string BaseJoint<T>::str() const
    {
        std::ostringstream res;
        res << "[" << this->getNumAgents() << "](";
        if (this->getNumAgents() > 0)
        {
            number ag;
            for (ag = 0; ag < this->getNumAgents() - 1; ++ag)
            {
                res << this->get(ag) << ", ";
            }
            res << this->get(ag);
        }
        res << ")";
        return res.str();
    }

} // namespace sdm

namespace std
{
    template <typename T>
    struct hash<sdm::BaseJoint<T>>
    {
        typedef sdm::BaseJoint<T> argument_type;
        typedef std::size_t result_type;
        result_type operator()(argument_type const &in) const
        {
            size_t size = in.size();
            size_t seed = 0;
            for (size_t i = 0; i < size; i++)
                //Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, in[i]);
            return seed;
        }
    };
}
