/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

int16_t inclinationDecigrees[XYZ_AXIS_COUNT];
int16_t EstG[XYZ_AXIS_COUNT];
int16_t smallAngle;

void imuInit(void);

void imuConfigure(uint16_t throttle_correction_angle);

//void imuResetAccelerationSum(void);

void imuUpdate();

/*TODO: what are those two function doing here?! */

int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value);
