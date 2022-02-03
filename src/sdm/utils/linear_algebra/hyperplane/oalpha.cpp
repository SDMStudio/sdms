#include <sdm/utils/linear_algebra/hyperplane/oalpha.hpp>

namespace sdm
{
    oAlpha::oAlpha(double default_value) : AlphaVector(default_value)
    {
    }

    double oAlpha::getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o)
    {
        auto iter = this->repr.find(o);
        if (iter == this->repr.end())
            return this->default_value;
        else
            return iter->second.getValueAt(x, o);
    }

    void oAlpha::setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, double value)
    {
        auto iter = this->repr.find(o);
        if (iter == this->repr.end())
            this->repr.emplace(o, bAlpha(default_value));
        else
            this->repr.at(o).setValueAt(x, o, value);
    }
} // namespace sdm
