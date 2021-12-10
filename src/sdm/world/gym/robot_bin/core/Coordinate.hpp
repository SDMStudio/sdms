#pragma once

#include <string>

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