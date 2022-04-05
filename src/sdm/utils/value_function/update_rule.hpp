#pragma once

//  ------------------------------------------------------------------------
// |            INCLUDE UPDATE OPERATORS interfaces                         |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_rule/vupdate_rule.hpp>
#include <sdm/utils/value_function/update_rule/qupdate_rule.hpp>

//  ------------------------------------------------------------------------
// |            INCLUDE VALUE UPDATE OPER1TORS IMPLEMENTATIONS              |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_rule/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/update_rule/vupdate/lb_tabular_update.hpp>
#include <sdm/utils/value_function/update_rule/vupdate/pwlc_update.hpp>

//  ------------------------------------------------------------------------
// |            INCLUDE Q-VALUE UDPATE OPERATORS IMPLEMENTATIONS            |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_rule/qupdate/tabular_qupdate.hpp>
#include <sdm/utils/value_function/update_rule/qupdate/pwlc_qupdate.hpp>
#include <sdm/utils/value_function/update_rule/qupdate/serial_pwlc_qupdate.hpp>

//  ------------------------------------------------------------------------
// |                     INCLUDE UPDATE REGISTRY                            |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_rule/registry.hpp>
