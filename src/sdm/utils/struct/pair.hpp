// Pair implementation -*- C++ -*-

// Copyright (C) 2001, 2002, 2003, 2004, 2005, 2006, 2007
// Free Software Foundation, Inc.
//
// This file is part of the GNU ISO C++ Library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along
// with this library; see the file COPYING.  If not, write to the Free
// Software Foundation, 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,
// USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU General Public License.

/*
 *
 * Copyright (c) 1994
 * Hewlett-Packard Company
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Hewlett-Packard Company makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 *
 * Copyright (c) 1996,1997
 * Silicon Graphics Computer Systems, Inc.
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Silicon Graphics makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 */

/** @file stl_Pair.h
 *  This is an internal header file, included by other library headers.
 *  You should not attempt to use it directly.
 */

#pragma once
#include <cstdlib>
#include <sstream>

using namespace std;

namespace sdm
{
    template <class _T1, class _T2>
    struct Pair
    {
        typedef _T1 first_type;
        typedef _T2 second_type;

        _T1 first;
        _T2 second;

        Pair()
            : first(), second() {}

        Pair(const _T1 &__a, const _T2 &__b)
            : first(__a), second(__b) {}

        template <class _U1, class _U2>
        Pair(const Pair<_U1, _U2> &__p)
            : first(__p.first),
              second(__p.second) {}

        std::string str() const
        {
            std::ostringstream res;
            res << "Pair{" << first << ", " << second << "}";
            return res.str();
        }

        friend std::ostream &operator<<(std::ostream &os, const Pair &my_pair)
        {
            os << my_pair.str();
            return os;
        }
    };

    /// Two pairs of the same type are equal iff their members are equal.
    template <class _T1, class _T2>
    inline bool
    operator==(const Pair<_T1, _T2> &__x, const Pair<_T1, _T2> &__y)
    {
        return __x.first == __y.first && __x.second == __y.second;
    }

    /// <http://gcc.gnu.org/onlinedocs/libstdc++/20_util/howto.html#Pairlt>
    template <class _T1, class _T2>
    inline bool
    operator<(const Pair<_T1, _T2> &__x, const Pair<_T1, _T2> &__y)
    {
        return __x.first < __y.first || (!(__y.first < __x.first) && __x.second < __y.second);
    }

    /// Uses @c operator== to find the result.
    template <class _T1, class _T2>
    inline bool
    operator!=(const Pair<_T1, _T2> &__x, const Pair<_T1, _T2> &__y)
    {
        return !(__x == __y);
    }

    /// Uses @c operator< to find the result.
    template <class _T1, class _T2>
    inline bool
    operator>(const Pair<_T1, _T2> &__x, const Pair<_T1, _T2> &__y)
    {
        return __y < __x;
    }

    /// Uses @c operator< to find the result.
    template <class _T1, class _T2>
    inline bool
    operator<=(const Pair<_T1, _T2> &__x, const Pair<_T1, _T2> &__y)
    {
        return !(__y < __x);
    }

    /// Uses @c operator< to find the result.
    template <class _T1, class _T2>
    inline bool
    operator>=(const Pair<_T1, _T2> &__x, const Pair<_T1, _T2> &__y)
    {
        return !(__x < __y);
    }

    template <typename _Tp>
    class reference_wrapper;

    // Helper which adds a reference to a type when given a reference_wrapper
    template <typename _Tp>
    struct __strip_reference_wrapper
    {
        typedef _Tp __type;
    };

    template <typename _Tp>
    struct __strip_reference_wrapper<reference_wrapper<_Tp>>
    {
        typedef _Tp &__type;
    };

    template <typename _Tp>
    struct __strip_reference_wrapper<const reference_wrapper<_Tp>>
    {
        typedef _Tp &__type;
    };

    template <typename _Tp>
    struct __decay_and_strip
    {
        typedef typename __strip_reference_wrapper<
            typename decay<_Tp>::type>::__type __type;
    };

    // NB: DR 706.
    template <class _T1, class _T2>
    inline Pair<typename __decay_and_strip<_T1>::__type,
                typename __decay_and_strip<_T2>::__type>
    make_pair(_T1 &&__x, _T2 &&__y)
    {
        return Pair<typename __decay_and_strip<_T1>::__type,
                    typename __decay_and_strip<_T2>::__type>(std::forward<_T1>(__x), std::forward<_T2>(__y));
    }

} // namespace sdm

namespace std
{
    template <typename T, typename U>
    struct hash<sdm::Pair<T, U>>
    {
        typedef sdm::Pair<T, U> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type  &in) const
        {
            size_t seed = 0;
            sdm::hash_combine(seed, in.first);
            sdm::hash_combine(seed, in.second);
            return seed;
        }
    };
}


namespace std
{
    template <typename T, typename U>
    struct hash<std::pair<T, U>>
    {
        typedef std::pair<T, U> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type  &in) const
        {
            size_t seed = 0;
            sdm::hash_combine(seed, in.first);
            sdm::hash_combine(seed, in.second);
            return seed;
        }
    };
}
