//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#ifndef ZEN_API_PROTOCOL_H_
#define ZEN_API_PROTOCOL_H_

#include <cstdint>

typedef enum EZenProtocolFunction : uint8_t
{
    ZenProtocolFunction_Invalid = 0,

    ZenProtocolFunction_Negotiate = 1,
    ZenProtocolFunction_Handshake = 2,
    ZenProtocolFunction_Execute = 3,
    ZenProtocolFunction_Get = 4,
    ZenProtocolFunction_Set = 5,
    ZenProtocolFunction_Ack = 6,
    ZenProtocolFunction_Result = 7,
    ZenProtocolFunction_Event = 8,

    ZenProtocolFunction_Max
} EZenProtocolFunction;

#endif