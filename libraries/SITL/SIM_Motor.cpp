/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simple electric motor simulator class
*/

#include "SIM_Motor.h"
#include <AP_Motors/AP_Motors.h>

using namespace SITL;

// calculate rotational accel and thrust for a motor
void Motor::calculate_forces(const Aircraft::sitl_input &input,
                             const float thrust_scale,
                             uint8_t motor_offset,
                             Vector3f &rot_accel,
                             Vector3f &thrust) const
{
    float motor_speed = constrain_float((input.servos[motor_offset+servo]-1100)/900.0, 0, 1);
    rot_accel.x = -radians(5000.0) * sinf(radians(angle)) * motor_speed;
    rot_accel.y =  radians(5000.0) * cosf(radians(angle)) * motor_speed;
    rot_accel.z = yaw_factor * motor_speed * radians(400.0);
    thrust(0, 0, motor_speed * thrust_scale); // newtons
    if (roll_servo >= 0) {
        float roll = constrain_float(roll_min + (input.servos[roll_servo]-1000)*0.001*(roll_max-roll_min), roll_min, roll_max);
        Matrix3f rotation;
        rotation.identity();
        rotation.rotate(Vector3f(radians(roll), 0, 0));
        rot_accel = rotation * rot_accel;
        thrust = rotation * thrust;
    }
    if (pitch_servo >= 0) {
        float pitch = constrain_float(pitch_min + (input.servos[pitch_servo]-1000)*0.001*(pitch_max-pitch_min), pitch_min, pitch_max);
        Matrix3f rotation;
        rotation.identity();
        rotation.rotate(Vector3f(0, radians(pitch), 0));
        rot_accel = rotation * rot_accel;
        thrust = rotation * thrust;
    }
}

