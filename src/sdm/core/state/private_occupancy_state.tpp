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
        const PrivateOccupancyState &v) : BaseOccupancyState<TState, TJointHistory_p>(v), agent_id_(v.getAgentId()), bimap_jhist_hash(v.bimap_jhist_hash), compact_private_ostate(v.compact_private_ostate)
    {
    }

    template <typename TState, typename TJointHistory_p>
    number PrivateOccupancyState<TState, TJointHistory_p>::getAgentId() const
    {
        return this->agent_id_;
    }

    template <typename TState, typename TJointHistory_p>
    std::shared_ptr<PrivateOccupancyState<TState, TJointHistory_p>> PrivateOccupancyState<TState, TJointHistory_p>::getPrivateOccupancyStateAt(const typename PrivateOccupancyState<TState, TJointHistory_p>::ihistory_type &ihistory) const
    {
        return this->compact_private_ostate.at(ihistory);
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
        std::hash<Joint<typename PrivateOccupancyState<TState, TJointHistory_p>::ihistory_type>> hash_function;
        for (const auto &pair_state_hist_prob : *this)
        {
            auto jhist = this->getHistory(pair_state_hist_prob.first);
            this->bimap_jhist_hash[jhist->getIndividualHistory(this->agent_id_)].insert(bimap_value(jhist, hash_function(jhist->getIndividualHistories())));
            this->compact_private_ostate[jhist->getIndividualHistory(this->agent_id_)].addProbability(pair_state_hist_prob.first, pair_state_hist_prob.second);
        }
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
