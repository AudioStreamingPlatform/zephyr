// Copyright (C) 2024 Bang & Olufsen A/S, Denmark
//
// SPDX-License-Identifier: LicenseRef-BnOProprietary

#ifndef TI_DT_BINDINGS_MSPM0_H
#define TI_DT_BINDINGS_MSPM0_H

// check <ti/driverlib/dl_timer.h>

// check DL_TIMER_TIMER_MODE
#define ONE_SHOT_DOWN    0
#define ONE_SHOT_UP      32
#define PERIODIC_DOWN    2
#define PERIODIC_UP      34
#define ONE_SHOT_UP_DOWN 16
#define PERIODIC_UP_DOWN 18

// check DL_TIMER_CLOCK_DIVIDE
#define RATE_1 0
#define RATE_2 1
#define RATE_3 2
#define RATE_4 3
#define RATE_5 4
#define RATE_6 5
#define RATE_7 6
#define RATE_8 7

#endif
