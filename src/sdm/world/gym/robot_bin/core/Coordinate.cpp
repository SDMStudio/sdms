#include <sstream>
#include <sdm/world/gym/robot_bin/core/Coordinate.hpp>

namespace sdm
{
    namespace world
    {
        namespace gym
        {
            Coordinate::Coordinate(int x, int y) : x(x), y(y)
            {
            }

            int Coordinate::getX() const
            {
                return this->x;
            }

            int Coordinate::getY() const
            {
                return this->y;
            }

            std::string Coordinate::str() const
            {
                std::ostringstream res;
                res << "Coord(" << x << ", " << y << ")";
                return res.str();
            }

            bool Coordinate::operator==(const Coordinate &coord)
            {
                return (this->getX() == coord.getX()) && (this->getY() == coord.getY());
            }
        }
    }
}