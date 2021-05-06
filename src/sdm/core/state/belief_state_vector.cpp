#include <sdm/core/state/belief_state_vector.hpp>

namespace sdm
{

    const double BeliefStateVector::PRECISION = 0.00001;
    // BeliefStateVector

    BeliefStateVector::BeliefStateVector()
    {
    }

    BeliefStateVector::BeliefStateVector(std::size_t size, double default_value) : Vector(size)
    {
        this->init(default_value);
    }

    BeliefStateVector::BeliefStateVector(const Vector &copy) : Vector(copy)
    {
    }

    BeliefStateVector::BeliefStateVector(const BeliefStateVector &copy) : Vector(copy)
    {
    }

    BeliefStateVector::BeliefStateVector(const std::vector<double> &vector) : Vector(vector.size())
    {
        number index = 0;
        for (const auto &value : vector)
        {
            this->setProbabilityAt(index, value);
            index++;
        }
    }

    BeliefStateVector::BeliefStateVector(std::initializer_list<double> init_list) : Vector(init_list.size())
    {
        std::initializer_list<double>::iterator it;
        number index = 0;
        for (it = init_list.begin(); it != init_list.end(); ++it, ++index)
        {
            this->setProbabilityAt(index, *it);
        }
    }

    void BeliefStateVector::setProbabilityAt(const number &index, double value)
    {
        (*this)[index] = value;
    }

    void BeliefStateVector::addProbabilityAt(const number &index, double value)
    {
        (*this)[index] += value;
    }

    std::string BeliefStateVector::str() const
    {
        std::ostringstream res;
        res << "[";
        for (number i = 0; i < this->size(); i++)
        {
            res << ((i == 0) ? "" : ", ");
            res << i << " --> " << (*this)[i];
        }
        res << "]";
        return res.str();
    }

    bool BeliefStateVector::operator==(const BeliefStateVector &other) const
    {
        if (this->size() != other.size())
        {
            return false;
        }
        for (number index = 0; index < this->size(); index++)
        {
            if (std::abs((*this)[index] - other[index]) > BeliefStateVector::PRECISION)
            {
                return false;
            }
        }
        return true;
    }

}