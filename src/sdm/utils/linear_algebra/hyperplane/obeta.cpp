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
} // namespace sdm
