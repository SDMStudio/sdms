#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/state/interface/history_interface.hpp>

namespace sdm
{
    class Hyperplane
    {
    public:
        Hyperplane(double default_value) : default_value(default_value) {}

        virtual bool isDominated(const Hyperplane &other) const = 0;

        virtual bool dominate(const Hyperplane &other) const
        {
            return other.isDominated(*this);
        }

        virtual double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o) const = 0;
        virtual double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u) const = 0;

        virtual void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, double value) = 0;
        virtual void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value) = 0;

        virtual size_t hash(double precision = -1) const { return 0;}
        virtual bool isEqual(const std::shared_ptr<Hyperplane> &other, double precision = -1) const { return this == other.get();}

        virtual std::string str() const
        {
            return "Hyperplane";
        }

        friend std::ostream &operator<<(std::ostream &os, const Hyperplane &plane)
        {
            os << plane.str();
            return os;
        }

    protected:
        double default_value;
    };
        
} 