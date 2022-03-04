#pragma once

//  ------------------------------------------------------------------------
// |            INCLUDE UPDATE OPERATORS interfaces                         |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>
#include <sdm/utils/value_function/update_operator/qupdate_operator.hpp>

//  ------------------------------------------------------------------------
// |            INCLUDE VALUE UPDATE OPER1TORS IMPLEMENTATIONS              |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_operator/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/lb_tabular_update.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/pwlc_update.hpp>

//  ------------------------------------------------------------------------
// |            INCLUDE Q-VALUE UDPATE OPERATORS IMPLEMENTATIONS            |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_operator/qupdate/tabular_qupdate.hpp>
#include <sdm/utils/value_function/update_operator/qupdate/pwlc_qupdate.hpp>
#include <sdm/utils/value_function/update_operator/qupdate/serial_pwlc_qupdate.hpp>

//  ------------------------------------------------------------------------
// |                     INCLUDE UPDATE REGISTRY                            |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_operator/registry.hpp>
