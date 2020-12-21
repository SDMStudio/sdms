#include <sdm/core/bounds/upper_bound.hpp>

namespace sdm
{

    template <typename TState>
    void UpperBound<TState>::addPoint(TState x, double y)
    {
        this->point_set.emplace(x, y);
    }

    template <typename TState>
    void UpperBound<TState>::addPoint(const Point<TState, double> &point)
    {
        this->point_set.emplace(point);
    }

    template <typename TState>
    void UpperBound<TState>::deletePoint(TState x, double y)
    {
        this->point_set.erase(Point(x, y));
    }

    template <typename TState>
    void UpperBound<TState>::deletePoint(const Point<TState, double> &point)
    {
        this->point_set.erase(Point(x, y));
    }

    template <typename TState>
    double UpperBound<TState>::getValueAt(TState x)
    {
        for (auto it = this->point_set.begin(); it != this->point_set.end(); ++it)
        {
            if (it->first == x)
            {
                return it->second;
            }
        }
        // return this->getSawtoothValueAt(o, this->point_sets[h]);
    }


  template<typename TState>
  double point_set<o_state, o_action>::getSawtooth( const std::shared_ptr<Vector>& frequency, const std::pair<std::shared_ptr<Vector>, std::shared_ptr<Vector>>& entry ){
    double max = std::numeric_limits<double>::min();
    for(state x=0; x<common::model->getNumStates(); ++x) if((*entry.first)[x]>0)
      max = std::max(max, (*frequency)[x] * (*entry.second)[x]);
    return max;
  }

    template <typename TState>
    number UpperBound<TState>::size()
    {
        return this->point_set.size();
    }

    template <typename TState>
    void UpperBound<TState>::prune()
    {
        auto entry = std::make_pair(s, v);
        auto reference = getSawtooth(s, entry);

        // if the sawtooth complement value is higher than zero, prune "entry"
        if (reference >= 0.0)
            return true;

        // else
        for (auto iter = this->container.at(h).begin(); iter != this->container.at(h).end(); iter++)
        {
            if (reference > getSawtooth(s, *iter))
                return true;
            else if (this->getSawtooth(iter->first, *iter) > this->getSawtooth(iter->first, entry))
            {
                this->container.at(h).erase(iter);
                this->container.at(h).insert(entry);
                return false;
            }
        }

        this->container.at(h).insert(entry);
        return false;
    }
    template <typename TState>
    void UpperBound<TState>::initialize(double value)
    {
        // dpomdp->getMaxReward() * (dpomdp->getPlanninsgHorizon() - h);

    }

} // namespace sdm