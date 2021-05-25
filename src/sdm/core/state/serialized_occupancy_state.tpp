#include <sdm/core/state/serialized_occupancy_state.hpp>

namespace sdm
{

    // template <typename TState, typename TJointHistory_p>
    // SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState() : OccupancyState<TState, TJointHistory_p>()
    // {
    //     this->agent = 0;
    // }


    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(double default_value) : OccupancyState<TState, TJointHistory_p>(default_value)
    {
        this->agent = 0;
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(number num_agents, double default_value) : OccupancyState<TState, TJointHistory_p>(num_agents, default_value)
    {
        this->agent = 0;
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(const SerializedOccupancyState &occupancy_state) : OccupancyState<TState, TJointHistory_p>(occupancy_state)
    {
        this->agent = occupancy_state.getCurrentAgentId();
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(const OccupancyState<TState, TJointHistory_p> &occupancy_state) : OccupancyState<TState, TJointHistory_p>(occupancy_state)
    {
        this->agent = 0;
    }

    template <typename TState, typename TJointHistory_p>
    number SerializedOccupancyState<TState, TJointHistory_p>::getCurrentAgentId() const
    {
        return this->agent;
    }

    template <typename TState, typename TJointHistory_p>
    void SerializedOccupancyState<TState, TJointHistory_p>::setAgent(number agent)
    {
        this->agent = agent;
    }

    template <typename TState, typename TJointHistory_p>
    typename SerializedOccupancyState<TState, TJointHistory_p>::state_type::state_type SerializedOccupancyState<TState, TJointHistory_p>::getHiddenState(const Pair<state_type, jhistory_type> &state) const
    {
        return state.first.getState();
    }

    template <typename TState, typename TJointHistory_p>
    std::vector<typename SerializedOccupancyState<TState, TJointHistory_p>::state_type::action_type> SerializedOccupancyState<TState, TJointHistory_p>::getAction(const Pair<state_type, jhistory_type> &state) const
    {
        return state.first.getAction();
    }

    template <typename TState, typename TJointHistory_p>
    std::shared_ptr<SerializedOccupancyState<TState, TJointHistory_p>> SerializedOccupancyState<TState, TJointHistory_p>::getptr()
    {
        return std::static_pointer_cast<SerializedOccupancyState<TState, TJointHistory_p>>(this->shared_from_this());
    }

    template <typename TState, typename TJointHistory_p>
    std::shared_ptr<SerializedOccupancyState<TState, TJointHistory_p>> SerializedOccupancyState<TState, TJointHistory_p>::getOneStepUncompressedOccupancy() const
    {
        return std::static_pointer_cast<SerializedOccupancyState<TState, TJointHistory_p>>(OccupancyState<TState, TJointHistory_p>::getOneStepUncompressedOccupancy());
    }

    template <typename TState, typename TJointHistory_p>
    std::shared_ptr<SerializedOccupancyState<TState, TJointHistory_p>> SerializedOccupancyState<TState, TJointHistory_p>::getFullyUncompressedOccupancy() const
    {
        return std::static_pointer_cast<SerializedOccupancyState<TState, TJointHistory_p>>(OccupancyState<TState, TJointHistory_p>::getFullyUncompressedOccupancy());
    }

    template <typename TState, typename TJointHistory_p>
    std::string SerializedOccupancyState<TState, TJointHistory_p>::str() const
    {

        number horizon;
        std::ostringstream res, tmp;
        std::unordered_map<TJointHistory_p, std::pair<double, MappedVector<TState, double>>> map;

        for (const auto &pair_s_o_proba : *this)
        {
            auto prob = pair_s_o_proba.second;
            auto jhistory = pair_s_o_proba.first.second;
            auto serial_hidden_state = pair_s_o_proba.first.first;

            if (map.find(jhistory) == map.end())
            {
                map.emplace(jhistory, std::make_pair(0, MappedVector<TState, double>()));
                horizon = jhistory->getDepth();
            }

            map[jhistory].first += prob;
            map[jhistory].second[serial_hidden_state] = prob;
        }

        for (const auto &pair_s_o_proba : *this)
        {
            auto jhistory = pair_s_o_proba.first.second;
            auto serial_hidden_state = pair_s_o_proba.first.first;
            map[jhistory].second[serial_hidden_state] /= map[jhistory].first;
        }

        res << "<serial-occupancy-state size=\"" << map.size() << "\" agent=\"" << this->agent << "\" horizon=\"" << horizon << "\">" << std::endl;
        for (const auto pair_o_pair_proba_belief : map)
        {
            auto joint_hist = pair_o_pair_proba_belief.first;
            res << "\t<joint-history value=\"" << *joint_hist << "\" proba=" << pair_o_pair_proba_belief.second.first << " belief=" << pair_o_pair_proba_belief.second.second << "/>" << std::endl;
        }
        res << "</serial-occupancy-state>" << std::endl;

        return res.str();
    }

    template <typename TState, typename TJointHistory_p>
    std::string SerializedOccupancyState<TState, TJointHistory_p>::str_hyperplan() const
    {

        std::ostringstream res, tmp;
        number horizon;
        std::unordered_set<TJointHistory_p> set;
        for (const auto &pair_x_o_p : *this)
        {
            set.emplace(pair_x_o_p.first.second);
            horizon = pair_x_o_p.first.second->getDepth();
        }

        res << "<serial-hyperplan size=\"" << set.size() << "\" agent=\"" << this->agent << "\" horizon=\"" << horizon << "\">" << std::endl;
        for (const auto joint_hist : set)
        {
            res << "\t<joint-history name=\"" << joint_hist->short_str() << "\" vector =" << MappedVector<Pair<TState, TJointHistory_p>, double>::str() << "/>" << std::endl;
        }
        res << "</serial-hyperplan>" << std::endl;

        return res.str();
    }

    template <typename TState, typename TJointHistory_p>
    const SerializedBeliefState SerializedOccupancyState<TState, TJointHistory_p>::createBelief(const TJointHistory_p &joint_history) const
    {
        SerializedBeliefState belief ; //= dynamic_cast<SerializedBeliefState&>(OccupancyState<TState, TJointHistory_p>::createBelief(joint_history));
        
        //Go over all hidden state conditionning to a joint history
        for (auto hidden_state : this->getStatesAt(joint_history))
        {
            belief.addProbabilityAt(hidden_state,this->at(std::make_pair(hidden_state, joint_history)));
        }

        belief.setAgent(this->getCurrentAgentId());
        return belief;
    }

    template <typename TState, typename TJointHistory_p>
    const SerializedBeliefState SerializedOccupancyState<TState, TJointHistory_p>::createBeliefWeighted(const TJointHistory_p &joint_history) const
    {
        auto belief = this->createBelief(joint_history);
        double sum = belief.norm_1();
        for (const auto &b_s : belief)
        {
            belief[b_s.first] = b_s.second / sum;
        }
        return belief;
    }

} // namespace sdm

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::SerializedOccupancyState<S, V>>
    {
        typedef sdm::SerializedOccupancyState<S, V> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::BaseOccupancyState<S, V>>()(in);
        }
    };
}