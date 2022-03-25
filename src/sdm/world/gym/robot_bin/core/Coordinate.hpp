#pragma once

#include <string>
#include <sdm/types.hpp>

namespace sdm
{
    namespace world
    {
        namespace gym
        {
            class Coordinate
            {
            public:
                Coordinate(int x, int y);

                int getX() const;
                int getY() const;

                std::string str() const;
                bool operator==(const Coordinate &coord);

                friend std::ostream &operator<<(std::ostream &os, const Coordinate &coord)
                {
                    os << coord.str();
                    return os;
                }

            protected:
                int x, y;
            };
        }
    }
}

namespace std
{
    template <>
    struct hash<sdm::world::gym::Coordinate>
    {
        inline std::size_t operator()(const sdm::world::gym::Coordinate &in, double precision) const
        {
            std::size_t seed = 0;
            sdm::hash_combine(seed, in.getX());
            return seed;
        }

        inline std::size_t operator()(const sdm::world::gym::Coordinate &in) const
        {
            return std::hash<sdm::world::gym::Coordinate>()(in, 0.);
        }
    };
}