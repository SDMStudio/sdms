
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/types.hpp>

namespace sdm
{

    template <class T0, class T1>
    void RecursiveMap<T0, T1>::recursive_emplace(T0 v0, T1 v1)
    {
        this->emplace(v0, v1);
    }

    template <class T0, class T1, class T2, class... Ts>
    void RecursiveMap<T0, T1, T2, Ts...>::recursive_emplace(T0 v0, T1 v1, T2 v2, Ts... vs)
    {
        if (this->find(v0) != this->end())
        {
            this->operator[](v0).recursive_emplace(v1, v2, vs...);
        }
        else
        {
            RecursiveMap_t<T1, T2, Ts...> sub_rmap;
            sub_rmap.recursive_emplace(v1, v2, vs...);
            this->emplace(v0, sub_rmap);
        }
    }

    template <class T0, class T1>
    T1 RecursiveMap<T0, T1>::operator()(T0 v0)
    {
        return this->operator[](v0);
    }

    template <class T0, class T1, class T2, class... Ts>
    auto RecursiveMap<T0, T1, T2, Ts...>::operator()(T0 v0, T1 v1, Ts... vs)
    {
        return this->operator[](v0)(v1, vs...);
    }

    template <class T0, class T1, class T2, class... Ts>
    RecursiveMap<T1, T2, Ts...> RecursiveMap<T0, T1, T2, Ts...>::operator()(T0 v0)
    {
        return this->operator[](v0);
    }

} // namespace sdm