#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
    class BeliefStateVector : public Vector
    {
    protected:
        static const double PRECISION;

    public:
        using state_type = number;
        using value_type = double;

        BeliefStateVector();
        BeliefStateVector(std::size_t, double);
        BeliefStateVector(const Vector &);
        BeliefStateVector(const BeliefStateVector &);
        BeliefStateVector(const std::vector<double> &);
        BeliefStateVector(std::initializer_list<double>);

        void setProbabilityAt(const number &, double);
        void addProbabilityAt(const number &, double);

        std::string str() const;

        bool operator==(const BeliefStateVector &) const;

        friend std::ostream &operator<<(std::ostream &os, BeliefStateVector &belief)
        {
            os << "<belief " << belief.str() << "/>";
            return os;
        }
    };
}

namespace std
{
    template <>
    struct hash<sdm::BeliefStateVector>
    {
        typedef sdm::BeliefStateVector argument_type;
        typedef std::size_t result_type;
        result_type operator()(argument_type const &in) const
        {
            return std::hash<sdm::Vector>()(in);
        }
    };
}
