#include <sdm/core/state/state.hpp>
#include <sdm/core/state/interface/joint_history_interface.hpp>

namespace sdm
{

    std::shared_ptr<JointHistoryInterface> HistoryInterface::toJointHistory()
    {
        return std::dynamic_pointer_cast<JointHistoryInterface>(this->getPointer());
    }

} // 