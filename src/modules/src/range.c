/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ranges.c: Centralize range measurements for different directions
 *           and make them available as log
 */
#include <stdint.h>

#include "log.h"

#include "range.h"
#include "stabilizer_types.h"
#include "estimator.h"

static uint16_t ranges[RANGE_T_END] = {0,};

void rangeSet(rangeDirection_t direction, float range_m)
{
  if (direction > (RANGE_T_END-1)) return;

  ranges[direction] = range_m * 1000;
}

float rangeGet(rangeDirection_t direction)
{
    if (direction > (RANGE_T_END-1)) return 0;

  return ranges[direction];
}

bool rangeEnqueueDownRangeInEstimator(float distance, float stdDev, uint32_t timeStamp) {
  tofMeasurement_t tofData;
  tofData.timestamp = timeStamp;
  tofData.distance = distance;
  tofData.stdDev = stdDev;

  return estimatorEnqueueTOF(&tofData);
}

bool rangeEnqueuePlaneDistanceInEstimator(planeDistanceMeasurement_t *plane) {
  float z = 0;
  // Populate z measurements
  // Datasheet says 4m is limit for range
  for (int i = 0; i<4; i++){
    z = (float)ranges[plane->directions[i]]/1000.0f;
    if(z< 3.75f && z> 0.1f)
      plane->z[i] = z;
    else
      plane->z[i] = 0;
  }
  return estimatorEnqueuePlaneDistance(plane);
}

LOG_GROUP_START(range)
LOG_ADD(LOG_UINT16, front, &ranges[rangeFront])
LOG_ADD(LOG_UINT16, back, &ranges[rangeBack])
LOG_ADD(LOG_UINT16, up, &ranges[rangeUp])
LOG_ADD(LOG_UINT16, left, &ranges[rangeLeft])
LOG_ADD(LOG_UINT16, right, &ranges[rangeRight])
LOG_ADD(LOG_UINT16, zrange, &ranges[rangeDown])
LOG_GROUP_STOP(range)
