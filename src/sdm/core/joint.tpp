#include <sdm/core/joint.hpp>

namespace sdm
{
    template <typename T>
    Joint<T>::Joint() : std::vector<T>() {}

    template <typename T>
    Joint<T>::Joint(const std::vector<T> &joint_item) : std::vector<T>(joint_item) {}

    template <typename T>
    Joint<T>::Joint(const std::vector<number> &, const std::vector<T> &joint_item) : std::vector<T>(joint_item) {}

    template <typename T>
    Joint<T>::Joint(std::initializer_list<T> list_values) : std::vector<T>(list_values) {}

    template <typename T>
    Joint<T>::~Joint() {}

    template <typename T>
    const T &Joint<T>::get(const number &index) const
    {
        return this->at(index);
    }

    template <typename T>
    number Joint<T>::getNumAgents() const
    {
        return this->size();
    }

    template <typename T>
    T Joint<T>::operator()(const number &i)
    {
        return (*this)[i];
    }

} // namespace sdm

namespace std
{
    template <typename T>
    struct hash<sdm::Joint<T>>
    {
        typedef sdm::Joint<T> argument_type;
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
