#pragma once

#include <sdm/core/state/occupancy_state_v2.hpp>

namespace sdm
{
  /**
   * @brief A private state of occupancy refers to a 
   * 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  class PrivateOccupancyState : public OccupancyState
  {
  public:

    PrivateOccupancyState();
    PrivateOccupancyState(double default_value);
    PrivateOccupancyState(number num_agents, double default_value);
    PrivateOccupancyState(number agent_id, number num_agents, double default_value);
    PrivateOccupancyState(const PrivateOccupancyState &);
    PrivateOccupancyState(const OccupancyState &);

    number getAgentId() const;
    const std::vector<std::shared_ptr<HistoryInterface>> &getPartialJointHistory(const std::shared_ptr<JointHistoryInterface> &) const;
    std::shared_ptr<JointHistoryInterface> getJointHistory(const std::vector<std::shared_ptr<HistoryInterface>> &) const;

    void finalize(bool = true);

    bool operator==(const PrivateOccupancyState &) const;

  protected:
    std::vector<std::shared_ptr<HistoryInterface>> getPartialJointHistory(const std::vector<std::shared_ptr<HistoryInterface>> &) const;

    /**
     * @brief The agent's identifier 
     */
    number agent_id_;

    typedef boost::bimaps::bimap<std::shared_ptr<JointHistoryInterface>, Joint<std::shared_ptr<HistoryInterface>>> bimap_type;
    typedef typename bimap_type::value_type bimap_value;

    /**
     * @brief Bimap that map joint histories and hash of o^{-i} 
     */
    bimap_type bimap_jhist_partial_jhist;
  };
} // namespace sdm
