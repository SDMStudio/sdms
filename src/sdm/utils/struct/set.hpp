#include <sdm/types.hpp>

#include <functional>
#include <unordered_set>

namespace sdm
{
    template <class T>
    struct equal_container
    {
        virtual bool operator()(const std::shared_ptr<T> &, const std::shared_ptr<T> &) const;
    };

    template <class T>
    struct hash_container
    {
        virtual size_t operator()(const std::shared_ptr<T> &) const;
    };

    template <class T>
    struct Set : public std::unordered_set<std::shared_ptr<T>, hash_container<T>, equal_container<T>>
    {
        Set();
        bool contains(std::shared_ptr<T> &);

        friend std::ostream &operator<<(std::ostream &os, const vector_container &container)
        {
            os << "<vector_container size=" << container.size() << ">" << std::endl;
            for (const auto &entry : container)
                os << "\t"
                   << "<belief values=" << *entry << " @=" << entry << ">" << std::endl;
            os << "</vector_container>" << std::endl;
            return os;
        }
    };
} // namespace sdm

#include <sdm/utils/struct/set.tpp>
