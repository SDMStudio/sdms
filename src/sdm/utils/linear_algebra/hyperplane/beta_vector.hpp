#pragma once

#include <sdm/macros.hpp>
#include <sdm/exception.hpp>
#include <sdm/utils/linear_algebra/hyperplane/hyperplane.hpp>

namespace sdm
{
    class BetaVector : public Hyperplane
    {
    public:
        static double PRECISION;

        BetaVector(double default_value) : Hyperplane(default_value) {}

        virtual double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u) const = 0;
        virtual void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value) = 0;
        
        double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o) const;
        void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, double value);
    };
}

DEFINE_STD_HASH(sdm::BetaVector, sdm::BetaVector::PRECISION);
