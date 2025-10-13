/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   Yaml.h
 * @brief  Tiny header to provide a shortcut in NS `mola::` to MRPT yaml class.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */
#pragma once

#include <mrpt/containers/yaml.h>

namespace mola
{
/** Convenient typedef to save typing in the MOLA project. */
using Yaml = mrpt::containers::yaml;

};  // namespace mola
