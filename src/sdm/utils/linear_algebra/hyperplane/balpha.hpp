#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/hyperplane/alpha_vector.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{
    class bAlpha : public AlphaVector
    {
    public:
        bAlpha(double default_value);

        bool isDominated(const Hyperplane &other) const;

        double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &) const;
        void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, double value);
        double getBetaValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &u, const std::shared_ptr<POMDPInterface> &pomdp, number t);

        virtual size_t hash(double precision = -1) const;
        virtual bool isEqual(const std::shared_ptr<Hyperplane> &other, double precision = -1) const;

        std::string str() const;

    protected:
        MappedVector<std::shared_ptr<State>, double> repr;
    };
}
