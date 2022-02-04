#pragma once

#include <sdm/macros.hpp>
#include <sdm/exception.hpp>
#include <sdm/utils/linear_algebra/hyperplane/hyperplane.hpp>
#include <sdm/world/base/pomdp_interface.hpp>

namespace sdm
{
    class AlphaVector : public Hyperplane
    {
    public:
        static double PRECISION;

        AlphaVector(double default_value) : Hyperplane(default_value) {}

        virtual double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o) const = 0;
        virtual void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, double value) = 0;
        virtual double getBetaValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &u, const std::shared_ptr<POMDPInterface> &pomdp, number t) = 0;

        double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u) const;
        void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value);
    };
}

DEFINE_STD_HASH(sdm::AlphaVector, sdm::AlphaVector::PRECISION);