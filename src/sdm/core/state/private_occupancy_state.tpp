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
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(
        std::size_t size, double default_value) : BaseOccupancyState<TState, TJointHistory_p>(size, default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(
        const PrivateOccupancyState &v) : BaseOccupancyState<TState, TJointHistory_p>(v)
    {
    }

    template <typename TState, typename TJointHistory_p>
    std::string PrivateOccupancyState<TState, TJointHistory_p>::str() const
    {
        std::ostringstream res, tmp;
        res << "<private-occupancy-state horizon='?'>" << std::endl;
        for (const auto pair_x_o_p : *this)
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
