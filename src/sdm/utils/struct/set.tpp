#include <sdm/utils/struct/set.hpp>

namespace sdm
{
    template <class T>
    size_t hash_container<T>::operator()(const std::shared_ptr<T> &arg) const
    {
        return std::hash<T>()(*arg);
    }

    template <class T>
    bool equal_container<T>::operator()(const std::shared_ptr<T> &left, const std::shared_ptr<T> &right) const
    {
        return (left == right) or left->operator==(right);
    }

    template <class T>
    Set<T>::vector_container() {}

    template <class T>
    bool Set<T>::contains(std::shared_ptr<T> &p_elem)
    {
        auto iterator = this->find(p_elem);
        if (iterator == this->end())
            return false;
        p_elem = *iterator;
        return true;
    }
}