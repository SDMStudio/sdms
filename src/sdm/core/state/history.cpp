#include <sdm/types.hpp>
#include <sdm/core/state/history.hpp>

namespace sdm
{
    // History::History(const std::vector<std::shared_ptr<Item>> &history) : history_(history)
    // {
    // }

    // History::History(const History &copy) : history_(copy.history_)
    // {
    // }

    // std::shared_ptr<HistoryInterface> History::next(const std::shared_ptr<Item> &item, bool backup)
    // {
    //     if (backup)
    //     {
    //         this->history_.push_back(item);
    //         return std::static_pointer_cast<HistoryInterface>(this->shared_from_this());
    //     }
    //     else
    //     {
    //         std::shared_ptr<History> copy = std::make_shared<History>(*this);
    //         return copy->next(item, true);
    //     }
    // }

    // std::shared_ptr<HistoryInterface> History::getPreviousHistory()
    // {
    //     return std::make_shared<History>(std::vector<std::shared_ptr<Item>>(this->history_.begin(), this->history_.end() - 1));
    // }

    // std::shared_ptr<Item> History::get(number t) const
    // {
    //     assert((t < this->getHorizon()));
    //     return this->history_.at(t);
    // }

    // number History::getHorizon() const
    // {
    //     return this->history_.size();
    // }

    // std::string History::str() const
    // {
    // }

    // TypeState History::getTypeState() const
    // {
    //     return TypeState::STATE;
    // }

} // namespace sdm
