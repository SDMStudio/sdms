/**
 * @file history.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief This file contains the generic history class
 * @version 0.1
 * @date 14/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <sdm/core/state/interface/history_interface.hpp>

namespace sdm
{
    // class History : public HistoryInterface
    // {
    // public:
    //     History(const History &copy);
    //     History(const std::vector<std::shared_ptr<Item>> &history = {});

    //     std::shared_ptr<HistoryInterface> next(const std::shared_ptr<Item> &item, bool backup = false);
    //     std::shared_ptr<HistoryInterface> getPreviousHistory();

    //     std::shared_ptr<Item> get(number t) const;
    //     number getHorizon() const;

    //     std::string str() const;

    // protected:
    //     std::vector<std::shared_ptr<Item>> history_;
    // };

    // class History : public History
    // {
    // public:
    //     History(const std::vector<std::shared_ptr<Observation>> &);
    //     History(const History &copy);

    //     std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action = nullptr, bool backup = true);

    //     std::shared_ptr<HistoryInterface> getPreviousHistory();

    //     std::shared_ptr<Action> getLastAction();
    //     std::shared_ptr<Observation> getLastObservation();

    //     std::shared_ptr<Action> getActionAt(number t);
    //     std::shared_ptr<Observation> getObservationAt(number t);

    //     std::string str() const;

    // protected:
    //     std::vector<std::shared_ptr<Observation>> history_;
    // };

} // namespace sdm