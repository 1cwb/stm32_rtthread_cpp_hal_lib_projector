/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "axis.hpp"
#include "maths.hpp"
#include "vector.hpp"

namespace bfimu {

#define ALIGNMENT_ROTATION_WIDTH 2

#define ALIGNMENT_YAW_ROTATIONS_POSITION   (0 * ALIGNMENT_ROTATION_WIDTH)
#define ALIGNMENT_PITCH_ROTATIONS_POSITION (1 * ALIGNMENT_ROTATION_WIDTH)
#define ALIGNMENT_ROLL_ROTATIONS_POSITION  (2 * ALIGNMENT_ROTATION_WIDTH)

#define ALIGNMENT_YAW_ROTATIONS_MASK (0x3 << ALIGNMENT_YAW_ROTATIONS_POSITION)
#define ALIGNMENT_PITCH_ROTATIONS_MASK (0x3 << ALIGNMENT_PITCH_ROTATIONS_POSITION)
#define ALIGNMENT_ROLL_ROTATIONS_MASK (0x3 << ALIGNMENT_ROLL_ROTATIONS_POSITION)

#define ALIGNMENT_AXIS_ROTATIONS_MASK(axis) (0x3 << ((FD_YAW - axis) * ALIGNMENT_ROTATION_WIDTH))

#define ALIGNMENT_YAW_ROTATIONS(bits) ((bits & ALIGNMENT_YAW_ROTATIONS_MASK) >> ALIGNMENT_YAW_ROTATIONS_POSITION)
#define ALIGNMENT_PITCH_ROTATIONS(bits) ((bits & ALIGNMENT_PITCH_ROTATIONS_MASK) >> ALIGNMENT_PITCH_ROTATIONS_POSITION)
#define ALIGNMENT_ROLL_ROTATIONS(bits) ((bits & ALIGNMENT_ROLL_ROTATIONS_MASK) >> ALIGNMENT_ROLL_ROTATIONS_POSITION)

#define ALIGNMENT_AXIS_ROTATIONS(bits, axis) ((bits & ALIGNMENT_AXIS_ROTATIONS_MASK(axis)) >> ((FD_YAW - axis) * ALIGNMENT_ROTATION_WIDTH))

// [1:0] count of 90 degree rotations from 0
// [3:2] indicates 90 degree rotations on pitch
// [5:4] indicates 90 degree rotations on roll
#define ALIGNMENT_TO_BITMASK(alignment) ((alignment - CW0_DEG) & 0x3) | (((alignment - CW0_DEG) & 0x4) << 1)
    
static constexpr float DEFAULT_ALIGN_BOARD_ROLL = 0.0f;
static constexpr float DEFAULT_ALIGN_BOARD_PITCH = 0.0f;
static constexpr float DEFAULT_ALIGN_BOARD_YAW = 0.0f;

typedef enum {
    ALIGN_DEFAULT = 0, // driver-provided alignment

    // the order of these 8 values also correlate to corresponding code in ALIGNMENT_TO_BITMASK.

                            // R, P, Y
    CW0_DEG = 1,            // 00,00,00
    CW90_DEG = 2,           // 00,00,01
    CW180_DEG = 3,          // 00,00,10
    CW270_DEG = 4,          // 00,00,11
    CW0_DEG_FLIP = 5,       // 00,10,00 // _FLIP = 2x90 degree PITCH rotations
    CW90_DEG_FLIP = 6,      // 00,10,01
    CW180_DEG_FLIP = 7,     // 00,10,10
    CW270_DEG_FLIP = 8,     // 00,10,11

    ALIGN_CUSTOM = 9,    // arbitrary sensor angles, e.g. for external sensors
} sensor_align_e;

#define SENSOR_ALIGNMENT(ROLL, PITCH, YAW) ((sensorAlignment_t){\
    .roll = DEGREES_TO_DECIDEGREES(ROLL), \
    .pitch = DEGREES_TO_DECIDEGREES(PITCH), \
    .yaw = DEGREES_TO_DECIDEGREES(YAW), \
})

#define CUSTOM_ALIGN_CW0_DEG         SENSOR_ALIGNMENT(  0,   0,   0)
#define CUSTOM_ALIGN_CW90_DEG        SENSOR_ALIGNMENT(  0,   0,  90)
#define CUSTOM_ALIGN_CW180_DEG       SENSOR_ALIGNMENT(  0,   0, 180)
#define CUSTOM_ALIGN_CW270_DEG       SENSOR_ALIGNMENT(  0,   0, 270)
#define CUSTOM_ALIGN_CW0_DEG_FLIP    SENSOR_ALIGNMENT(  0, 180,   0)
#define CUSTOM_ALIGN_CW90_DEG_FLIP   SENSOR_ALIGNMENT(  0, 180,  90)
#define CUSTOM_ALIGN_CW180_DEG_FLIP  SENSOR_ALIGNMENT(  0, 180, 180)
#define CUSTOM_ALIGN_CW270_DEG_FLIP  SENSOR_ALIGNMENT(  0, 180, 270)

class Alignment {
public:
    Alignment()
    {

    }
    ~Alignment(){}

    static void boardAlignemntInit(float rollDegrees, float pitchDegrees, float yawDegrees)
    {
        Alignment::rollDegrees  = rollDegrees;
        Alignment::pitchDegrees = pitchDegrees;
        Alignment::yawDegrees   = yawDegrees;

        if (isBoardAlignmentStandard()) {
            return;
        }

        standardBoardAlignment = false;

        fp_angles_t rotationAngles;
        rotationAngles.angles.roll  = degreesToRadians(rollDegrees );
        rotationAngles.angles.pitch = degreesToRadians(pitchDegrees);
        rotationAngles.angles.yaw   = degreesToRadians(yawDegrees  );

        Matrix33::buildRotationMatrix(boardRotation, rotationAngles);
    }

    static bool isBoardAlignmentStandard()
    {
        return rollDegrees==0.0f && pitchDegrees==0.0f && yawDegrees==0.0f;
    }

    static void alignSensorViaMatrix(Vector3 &dest, float roll, float pitch, float yaw)
    {
        Matrix33 rm;
        buildRotationMatrixFromAngles(rm, roll, pitch, yaw);
        applyRotationMatrix(dest, rm);
    }

    static void alignSensorViaRotation(Vector3 &dest, sensor_align_e rotation)
    {
        const Vector3 tmp = dest;

        switch (rotation) {
        default:
        case CW0_DEG:
            dest.x() = tmp.x();
            dest.y() = tmp.y();
            dest.z() = tmp.z();
            break;
        case CW90_DEG:
            dest.x() = tmp.y();
            dest.y() = -tmp.x();
            dest.z() = tmp.z();
            break;
        case CW180_DEG:
            dest.x() = -tmp.x();
            dest.y() = -tmp.y();
            dest.z() = tmp.z();
            break;
        case CW270_DEG:
            dest.x() = -tmp.y();
            dest.y() = tmp.x();
            dest.z() = tmp.z();
            break;
        case CW0_DEG_FLIP:
            dest.x() = -tmp.x();
            dest.y() = tmp.y();
            dest.z() = -tmp.z();
            break;
        case CW90_DEG_FLIP:
            dest.x() = tmp.y();
            dest.y() = tmp.x();
            dest.z() = -tmp.z();
            break;
        case CW180_DEG_FLIP:
            dest.x() = tmp.x();
            dest.y() = -tmp.y();
            dest.z() = -tmp.z();
            break;
        case CW270_DEG_FLIP:
            dest.x() = -tmp.y();
            dest.y() = -tmp.x();
            dest.z() = -tmp.z();
            break;
        }

        if (!standardBoardAlignment) {
            alignBoard(dest);
        }
    }


private:
    static void alignBoard(Vector3 &vec)
    {
        applyRotationMatrix(vec, boardRotation);
    }
    static void buildRotationMatrixFromAngles(Matrix33 &rm, float roll, float pitch, float yaw)
    {
        fp_angles_t rotationAngles;
        rotationAngles.angles.roll  = DECIDEGREES_TO_RADIANS(roll);
        rotationAngles.angles.pitch = DECIDEGREES_TO_RADIANS(pitch);
        rotationAngles.angles.yaw   = DECIDEGREES_TO_RADIANS(yaw);

        Matrix33::buildRotationMatrix(rm, rotationAngles);
    }

    static void buildAlignmentFromStandardAlignment(float &roll, float &pitch, float &yaw, sensor_align_e stdAlignment)
    {
        if (stdAlignment == ALIGN_CUSTOM || stdAlignment == ALIGN_DEFAULT) {
            return;
        }

        uint8_t alignmentBits = ALIGNMENT_TO_BITMASK(stdAlignment);

        roll = DEGREES_TO_DECIDEGREES(90) * ALIGNMENT_AXIS_ROTATIONS(alignmentBits, 0);
        pitch = DEGREES_TO_DECIDEGREES(90) * ALIGNMENT_AXIS_ROTATIONS(alignmentBits, 1);
        yaw = DEGREES_TO_DECIDEGREES(90) * ALIGNMENT_AXIS_ROTATIONS(alignmentBits, 2);
    }
    static void alignSensorViaMatrix(Vector3 &dest, Matrix33 &sensorRotationMatrix)
    {
        applyRotationMatrix(dest, sensorRotationMatrix);
        if (!standardBoardAlignment) {
            alignBoard(dest);
        }
    }
private:
    static bool standardBoardAlignment;
    static Matrix33 boardRotation;
    static float rollDegrees;
    static float pitchDegrees;
    static float yawDegrees;
};

}