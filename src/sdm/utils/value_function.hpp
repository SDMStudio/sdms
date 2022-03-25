#pragma once

//  ------------------------------------------------------------------------
// |            INCLUDE UPDATE OPERATORS interfaces                         |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/value_function_interface.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/value_function/prunable_structure.hpp>

//  ------------------------------------------------------------------------
// |                INCLUDE VALUE FUNCTION IMPLEMENTATIONS                  |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/vfunction/tabular_vf_interface.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/vfunction/sawtooth_value_function.hpp>
#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>

//  ------------------------------------------------------------------------
// |               INCLUDE Q-VALUE FUNCTION IMPLEMENTATIONS                 |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/qfunction/tabular_q_interface.hpp>
#include <sdm/utils/value_function/qfunction/tabular_qvalue_function.hpp>
#include <sdm/utils/value_function/qfunction/pwlc_qvalue_function.hpp>
#include <sdm/utils/value_function/qfunction/deep_qvalue_function.hpp>

//  ------------------------------------------------------------------------
// |                INCLUDE VALUE FUNCTION REGISTRY                         |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/registry.hpp>
