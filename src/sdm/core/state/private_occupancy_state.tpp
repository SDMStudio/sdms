#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState()
    {
    }

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(
        double default_value) : BaseOccupancyState<TState, TJointHistory_p>(default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(number ag_id, double default_value) : BaseOccupancyState<TState, TJointHistory_p>(default_value), agent_id_(ag_id)
    {
    }

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(
        std::size_t size, double default_value) : BaseOccupancyState<TState, TJointHistory_p>(size, default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(
        const PrivateOccupancyState &v) : BaseOccupancyState<TState, TJointHistory_p>(v), agent_id_(v.getAgentId()), bimap_jhist_partial_jhist(v.bimap_jhist_partial_jhist)
    {
    }

    template <typename TState, typename TJointHistory_p>
    number PrivateOccupancyState<TState, TJointHistory_p>::getAgentId() const
    {
        return this->agent_id_;
    }

    template <typename TState, typename TJointHistory_p>
    std::string PrivateOccupancyState<TState, TJointHistory_p>::str() const
    {
        std::ostringstream res, tmp;
        res << "<private-occupancy-state horizon='?'>" << std::endl;
        for (const auto &pair_x_o_p : *this)
        {
            auto joint_hist = pair_x_o_p.first.second;

            res << "\t<probability state=\"" << pair_x_o_p.first.first << "\">" << std::endl;
            for (auto ihist : pair_x_o_p.first.second->getIndividualHistories())
            {
                res << tools::addIndent(ihist->str(), 2);
            }
            res << "\t\t" << pair_x_o_p.second << std::endl;
            res << "\t<probability>" << std::endl;
        }
        res << "</private-occupancy-state>" << std::endl;

        return res.str();
    }

    template <typename TState, typename TJointHistory_p>
    void PrivateOccupancyState<TState, TJointHistory_p>::finalize()
    {
        BaseOccupancyState<TState, TJointHistory_p>::finalize();

        // Add elements in bimap jhistory <--> jhistory^{-i}
        for (const auto &pair_state_hist_prob : *this)
        {
            auto jhist = this->getHistory(pair_state_hist_prob.first);
            this->bimap_jhist_partial_jhist.insert(bimap_value(jhist, jhist->getIndividualHistories()));
        }
    }

    template <typename TState, typename TJointHistory_p>
    bool PrivateOccupancyState<TState, TJointHistory_p>::operator==(const PrivateOccupancyState<TState, TJointHistory_p> &other) const
    {
        double ratio = -1;
        if (this->size() != other.size()){
            return false;
        }
        for (const auto &pair_partial_joint_history_joint_history : this->bimap_jhist_partial_jhist.right)
        {
            auto current_joint_history = pair_partial_joint_history_joint_history.second;
            auto partial_joint_history = pair_partial_joint_history_joint_history.first;
            auto other_joint_history = other.bimap_jhist_partial_jhist.right.at(partial_joint_history);

            for (const auto &hidden_state : this->getStatesAt(current_joint_history))
            {
                auto current_pair_state_joint_history = std::make_pair(hidden_state, current_joint_history);
                auto other_pair_state_joint_history = std::make_pair(hidden_state, other_joint_history);
                auto current_value = this->at(current_pair_state_joint_history);
                auto other_value = this->at(other_pair_state_joint_history);

                if (other_value == 0)
                {
                    return false;
                }
                if (ratio < 0)
                {
                    ratio = current_value / other_value;
                }
                else if (ratio - (current_value / other_value) > this->precision){
                    return false;
                } 
            }
        }
        return true;
    }

} // namespace sdm

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::PrivateOccupancyState<S, V>>
    {
        typedef sdm::PrivateOccupancyState<S, V> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::BaseOccupancyState<S, V>>()(in);
        }
    };
}
