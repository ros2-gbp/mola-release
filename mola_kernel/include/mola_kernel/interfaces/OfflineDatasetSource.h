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
 * @file   OfflineDatasetSource.h
 * @brief  Virtual interface for offline dataset sources
 * @author Jose Luis Blanco Claraco
 * @date   Dec 5, 2023
 */
#pragma once

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <cstdlib>

namespace mola
{
/** We reuse mrpt::poses::CPose3DInterpolator as a time-indexed map of SE(3)
 * poses */
using trajectory_t = mrpt::poses::CPose3DInterpolator;

/** Virtual base for offline dataset sources
 * \ingroup mola_kernel_interfaces_grp */
class OfflineDatasetSource
{
 public:
  OfflineDatasetSource() = default;

  /** @name Virtual interface of any OfflineDatasetSource
   *{ */

  /** Number of different time steps available to call getObservations() */
  virtual size_t datasetSize() const = 0;

  /** Returns the set of observations available for the given time step. */
  virtual mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(size_t timestep) const = 0;

  /** Returns true if a groundtruth is available
   *  for the vehicle trajectory.
   *  \sa getGroundTruthTrajectory()
   */
  virtual bool hasGroundTruthTrajectory() const { return false; }

  /** If hasGroundTruthTrajectory() returns true, this returns the dataset
   *  groundtruth for the vehicle trajectory.
   *
   *  Note that timestamps for datasets are not wall-clock time ("now"), but
   *  old timestamps of when original observations were grabbed.
   *
   *  \sa hasGroundTruthTrajectory()
   */
  virtual trajectory_t getGroundTruthTrajectory() const { return {}; }

  /** @} */
};

}  // namespace mola
