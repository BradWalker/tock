// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! True random number generator

use kernel::utilities::StaticRef;
use stm32h7xx::trng::RngRegisters;

pub(crate) const RNG_BASE: StaticRef<RngRegisters> =
    unsafe { StaticRef::new(0x4802_1800 as *const RngRegisters) };
