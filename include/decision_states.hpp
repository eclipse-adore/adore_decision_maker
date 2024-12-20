/********************************************************************************
 * Copyright (C) 2024-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/

#pragma once

namespace adore
{
enum ConditionFlags
{
  GNSS_OK                    = 1 << 0,
  VEHICLE_STATE_OK           = 1 << 1,
  SAFETY_CORRIDOR_PRESENT    = 1 << 2,
  WAYPOINTS_AVAILABLE        = 1 << 3,
  REFERENCE_TRAJECTORY_VALID = 1 << 4,
  ROUTE_AVAILABLE            = 1 << 5,
  LOCAL_MAP_AVAILABLE        = 1 << 6,
  NEED_ASSISTANCE            = 1 << 7
};

enum DecisionState
{
  EMERGENCY_STOP,
  FOLLOW_REFERENCE,
  FOLLOW_ROUTE,
  STANDSTILL,
  REMOTE_OPERATION,
  SAFETY_CORRIDOR,
  REQUESTING_ASSISTANCE
};

struct StateRequirement
{
  DecisionState state;
  int           required_conditions;
};


} // namespace adore