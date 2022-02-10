#include <sdm/utils/linear_algebra/hyperplane/obeta.hpp>

namespace sdm
{
    oBeta::oBeta(double default_value) : BetaVector(default_value)
    {
    }
    bool oBeta::isDominated(const Hyperplane &other) const
    {
        for (const auto &o_x_u_v : this->repr)
        {
            for (const auto &x_u_v : o_x_u_v.second)
            {
                for (const auto &u_v : x_u_v.second)
                {
                    if (u_v.second > other.getValueAt(x_u_v.first, o_x_u_v.first, u_v.first))
                        return false;
                }
            }
        }
        return true;
    }

    double oBeta::getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u) const
    {
        auto iter = this->repr.find(o);
        if (iter == this->repr.end())
            return this->default_value;
        else
            return iter->second.getValueAt(x, u);
    }

    void oBeta::setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value)
    {
        auto iter = this->repr.find(o);
        if (iter == this->repr.end())
            this->repr.emplace(o, MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Action>, double>(default_value));
        else
            this->repr.at(o).setValueAt(x, u, value);
    }

    size_t oBeta::hash(double precision) const
    {
        return 0;
    }

    bool oBeta::isEqual(const oBeta &other, double precision) const
    {
        if (other.size() > this->size())
        {
            return other.isEqual(*this, precision);
        }

        if (precision < 0)
            precision = oBeta::PRECISION;

        for (const auto &o_matrix : this->repr)
        {
            for (const auto &state_vector : o_matrix.second)
            {
                for (const auto &action_value : state_vector.second)
                {
                    if (std::abs(action_value.second - other.getValueAt(state_vector.first, o_matrix.first, action_value.first)) > precision)
                        return false;
                }
            }
        }
        return true;
    }

    bool oBeta::isEqual(const std::shared_ptr<Hyperplane> &other, double precision) const
    {
        auto other_oalpha = std::static_pointer_cast<oBeta>(other);
        if (other_oalpha == nullptr)
            return false;
        else
            return this->isEqual(*other_oalpha, precision);
    }

    size_t oBeta::size() const
    {
        return this->repr.size();
    }

    std::string oBeta::str() const
    {
        std::ostringstream res;
        res << "<plan>" << std::endl;

        for (const auto &hist_matrix : this->repr)
        {
            res << "\t<" << hist_matrix.first->short_str() << ">" << std::endl;
            for (const auto &state_vector : hist_matrix.second)
            {
                for (const auto &action_value : state_vector.second)
                {
                    res << "\t\t" << state_vector.first->str() << ", " << action_value.first << " : " << action_value.second << std::endl;
                }
            }
            res << "\t</>" << std::endl;
        }

        res << "</plan>" << std::endl;
        return res.str();
    }

} // namespace sdm
