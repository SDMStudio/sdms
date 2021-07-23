#include <sdm/utils/value_function/hierarchical_qvalue_function_v2.hpp>

// #include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{
    HierarchicalQValueFunctionV2::HierarchicalQValueFunctionV2(number horizon, double learning_rate, std::shared_ptr<QInitializer> initializer, double ball_r, bool keep_map)
        : QValueFunction(horizon), horizon_(horizon), learning_rate_(learning_rate), initializer_(initializer), ball_r_(ball_r), keep_map_(keep_map)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
        this->num_states_ = 0;
        // this->num_states_vector_ = std::vector<int>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, 0);
        this->closest_s_map_ = std::vector<std::unordered_map<std::shared_ptr<OccupancyStateInterface>, std::shared_ptr<OccupancyStateInterface>>>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, std::unordered_map<std::shared_ptr<OccupancyStateInterface>, std::shared_ptr<OccupancyStateInterface>>());
    }

    HierarchicalQValueFunctionV2::HierarchicalQValueFunctionV2(number horizon, double learning_rate, double default_value, double ball_r, bool keep_map) : HierarchicalQValueFunctionV2(horizon, learning_rate, std::make_shared<ValueInitializer>(default_value), ball_r, keep_map)
    {
        
    }

    void HierarchicalQValueFunctionV2::initialize()
    {

    }

    void HierarchicalQValueFunctionV2::initialize(double default_value, number t)
    {

    }

    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> HierarchicalQValueFunctionV2::getQValuesAt(const std::shared_ptr<State> &state, number t)
    {
        throw sdm::exception::NotImplementedException();
    }

    double HierarchicalQValueFunctionV2::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        throw sdm::exception::NotImplementedException();
    }

    double HierarchicalQValueFunctionV2::getQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        // std::cout << "HierarchicalQValueFunctionV2::getQValueAt()" << std::endl;
        auto h = this->isInfiniteHorizon() ? 0 : t;
        auto s_ = this->getClosestS(s, t);
        // this->initializeIfNeeded(s, s_, t);
        return this->representation[h].at(s_).getQValueAt(o, u, t);

    }

    void HierarchicalQValueFunctionV2::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta)
    {
        throw sdm::exception::NotImplementedException();
    }

    void HierarchicalQValueFunctionV2::updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t, double delta)
    {
        // std::cout << "HierarchicalQValueFunctionV2::updateQValueAt()" << std::endl;
        auto h = this->isInfiniteHorizon() ? 0 : t;
        auto s_ = this->getClosestS(s, t);
        this->representation[h].at(s_).updateQValueAt(o, u, t, delta);
        this->initializeIfNeeded(s, s_, t);
    }

    void HierarchicalQValueFunctionV2::updateQValueAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    bool HierarchicalQValueFunctionV2::isNotSeen(const std::shared_ptr<State> &state, number t)
    {
        return false;
    }

    int HierarchicalQValueFunctionV2::getNumStates() const
    {
        return this->num_states_;
    }

    void HierarchicalQValueFunctionV2::initializeIfNeeded(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<OccupancyStateInterface> &s_, number t)
    {   
        // std::cout << "HierarchicalQValueFunctionV2::initializeIfNeeded()" << std::endl;
        auto h = this->isInfiniteHorizon() ? 0 : t;

        if (this->keep_map_)
        {
            if (this->closest_s_map_[h].find(s) == this->closest_s_map_[h].end())
            {
                if (this->areInTheSameBall(s, s_, t))
                {
                    this->closest_s_map_[h].emplace(s, s_);
                }
                else
                {
                    this->initializeQValueFunctionAtWith(s, t, s_);
                    this->closest_s_map_[h].emplace(s, s);
                }
            }
        }
        else
        {
            if (!this->areInTheSameBall(s, s_, t))
            {
                this->initializeQValueFunctionAtWith(s, t, s_);
            }
        }
        
    }

    void HierarchicalQValueFunctionV2::initializeQValueFunctionAtWith(const std::shared_ptr<OccupancyStateInterface> &s, number t, const std::shared_ptr<OccupancyStateInterface> &s_)
    {   
        // std::cout << "HierarchicalQValueFunctionV2::initializeQValueFunctionAtWith()" << std::endl;
        auto h = this->isInfiniteHorizon() ? 0 : t;
  
        this->representation[h].emplace(s, TabularQValueFunction(0, learning_rate_, initializer_));
        this->representation[h].at(s) = this->representation[h].at(s_);
        this->num_states_++;
        
        
    }

    void HierarchicalQValueFunctionV2::initializeToZeroQValueFunctionAt(const std::shared_ptr<OccupancyStateInterface> &s, number t)
    {   
        // std::cout << "HierarchicalQValueFunctionV2::initializeToZeroQValueFunctionAt()" << std::endl;
        auto h = this->isInfiniteHorizon() ? 0 : t;
  
        this->representation[h].emplace(s, TabularQValueFunction(0, learning_rate_, initializer_));
        this->representation[h].at(s).initialize(0);
        this->num_states_++;
        this->closest_s_map_[h].emplace(s, s);
        
    }

    std::shared_ptr<OccupancyStateInterface> HierarchicalQValueFunctionV2::getClosestS(const std::shared_ptr<OccupancyStateInterface> &s, number t)
    {
        // std::cout << "HierarchicalQValueFunctionV2::getClosestS()" << std::endl;
        // std::cout << "this->keep_map_ " << this->keep_map_ << std::endl;
        auto h = this->isInfiniteHorizon() ? 0 : t;

        // //
        // if (this->keep_map_)
        // {
        //     std::cout << "a " << std::endl;
        //     // If s already has a label
        //     if (this->closest_s_map_[h].find(s) != this->closest_s_map_[h].end())
        //     {
        //         std::cout << "b " << std::endl;
        //         std::cout << *this->closest_s_map_[h].find(s) << std::endl;
        //         return this->closest_s_map_[h].find(s)->second;
        //     }
        // }
        // // If t=h is empty, s is the very first one to arrive.
        // else if (this->representation[h].size() == 0)
        // {
        //     std::cout << "c " << std::endl;
        //     this->initializeToZeroQValueFunctionAt(s, t);
        //     return s;
        // }
        // else
        // {
        //     std::cout << "d " << std::endl;
        //     double smallest_distance = 10000.0;
        //     std::shared_ptr<OccupancyStateInterface> closest_s;
            
        //     for (auto const& [s_, q_] : this->representation[h])
        //     {
        //         double distance = s_->minus(s);
        //         if (distance < smallest_distance)
        //         {
        //             smallest_distance = distance;
        //             closest_s = s_;
        //         }
        //     }
        //     return closest_s;
        // }



        // If s already has a label
        if (this->closest_s_map_[h].find(s) != this->closest_s_map_[h].end())
        {
            return this->closest_s_map_[h].find(s)->second;
        }
        // If t=h is empty, s is the very first one to arrive.
        else if (this->representation[h].size() == 0)
        {
            // std::cout << "A" << std::endl;
            this->initializeToZeroQValueFunctionAt(s, t);
            return s;
        }
        // // If s already has a q.
        // else if (this->representation[h].find(s) != this->representation[h].end())
        // {
        //     // std::cout << "B" << std::endl;
        //     return s;
        // }
        else
        {
            // std::cout << "C" << std::endl;
            double smallest_distance = 10000.0;
            std::shared_ptr<OccupancyStateInterface> closest_s;
            
            for (auto const& [s_, q_] : this->representation[h])
            {
                // std::cout << *s_ << std::endl;
                double distance = s_->minus(s);
                // std::cout << "distance " << distance << std::endl;
                if (distance < smallest_distance)
                {
                    smallest_distance = distance;
                    closest_s = s_;
                }
            }
            // std::cout << closest_s << std::endl;
            return closest_s;
        }

    }

    bool HierarchicalQValueFunctionV2::areInTheSameBall(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<OccupancyStateInterface> &s_, number t)
    {
        // std::cout << "HierarchicalQValueFunctionV2::areInTheSameBall()" << std::endl;
        return (s_->minus(s) < this->ball_r_);
    }

    std::string HierarchicalQValueFunctionV2::str() const
    {
        std::ostringstream res;
        res << "<hierarchical_qvalue_function_v2 horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (sdm::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << 0 << "\">" << std::endl;
            for (auto const& [s, q] : this->representation[i])
            {
                auto jh = *s->getJointHistories().begin();
                if (jh->getHorizon() < this->horizon_)
                {
                    res << "\t\t<S-Q>" << std::endl;
                    tools::indentedOutput(res, s->str().c_str(), 3);
                    res << std::endl;
                    tools::indentedOutput(res, q.str().c_str(), 3);
                    res << "\t\t</S-Q>" << std::endl;
                }
            }
            res << "\t</timestep>" << std::endl;
        }
        res << "</hierarchical_qvalue_function_v2>" << std::endl;
        return res.str();
    }
} // namespace sdm