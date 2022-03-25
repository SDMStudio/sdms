#pragma once

//  ------------------------------------------------------------------------
// |            INCLUDE ACTION SELECTION INTERFACES                         |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/action_selection/action_selection_interface.hpp>

//  ------------------------------------------------------------------------
// |                   INCLUDE ACTION SELECTION BASES                       |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/action_selection/action_selection_base.hpp>
#include <sdm/utils/value_function/action_selection/action_maxplan_base.hpp>

//  ------------------------------------------------------------------------
// |               INCLUDE ACTION SELECTION IMPLEMENTATIONS                 |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>
#include <sdm/utils/value_function/action_selection/action_maxplan_serial.hpp>


//  ------------------------------------------------------------------------
// |          INCLUDE CPLEX ACTION SELECTION IMPLEMENTATIONS                |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/action_selection/lp/action_maxplan_lp.hpp>
#include <sdm/utils/value_function/action_selection/lp/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/action_selection/lp/action_sawtooth_lp_serial.hpp>

//  ------------------------------------------------------------------------
// |           INCLUDE WCSP ACTION SELECTION IMPLEMENTATIONS                |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/action_selection/wcsp/action_maxplan_wcsp.hpp>
#include <sdm/utils/value_function/action_selection/wcsp/action_sawtooth_wcsp.hpp>

//  ------------------------------------------------------------------------
// |                     INCLUDE UPDATE REGISTRY                            |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/action_selection/registry.hpp>
