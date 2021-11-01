/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package Ftc2022FreightFrenzy_3543;

import com.qualcomm.robotcore.hardware.DcMotor;

import TrcCommonLib.trclib.TrcPose2D;

public class RobotInfo
{
    //
    // Hardware names.
    //
    static final String HWNAME_IMU                              = "imu";
    static final String HWNAME_WEBCAM                           = "Webcam 1";
    static final String HWNAME_BLINKIN                          = "blinkin";
    static final String HWNAME_LEFT_FRONT_WHEEL                 = "lfWheel";
    static final String HWNAME_RIGHT_FRONT_WHEEL                = "rfWheel";
    static final String HWNAME_LEFT_BACK_WHEEL                  = "lbWheel";
    static final String HWNAME_RIGHT_BACK_WHEEL                 = "rbWheel";
    static final String HWNAME_ARM                              = "arm";
    static final String HWNAME_INTAKE                           = "intakeMotor";
    static final String HWNAME_SPINNER                          = "spinnerMotor";
    static final String HWNAME_YLEFT_ODW_DEPLOYOR               = "yLeftOdwServo";
    static final String HWNAME_YRIGHT_ODW_DEPLOYOR              = "yRightOdwServo";
    static final String HWNAME_X_ODW_DEPLOYOR                   = "xOdwServo";
    //
    // Robot base odometry.
    //
    static final double ROBOT_LENGTH                            = 17.0;
    static final double ROBOT_WIDTH                             = 17.0;

    static final double FULL_FIELD_INCHES                       = 141.0;
    static final double HALF_FIELD_INCHES                       = FULL_FIELD_INCHES/2.0;
    static final double QUAD_FIELD_INCHES                       = FULL_FIELD_INCHES/4.0;
    static final double FULL_TILE_INCHES                        = 23.75;
    static final double HALF_TILE_INCHES                        = FULL_TILE_INCHES/2.0;

    static final double STARTPOS_FROM_FIELDCENTER_Y             = HALF_FIELD_INCHES - ROBOT_LENGTH/2.0;
    static final double STARTPOS_FROM_FIELDCENTER_X1            = QUAD_FIELD_INCHES;
    static final double STARTPOS_FROM_FIELDCENTER_X2            = HALF_TILE_INCHES;

    static final TrcPose2D STARTPOS_RED_1                       =
        new TrcPose2D(-STARTPOS_FROM_FIELDCENTER_X1, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    static final TrcPose2D STARTPOS_RED_2                       =
        new TrcPose2D(STARTPOS_FROM_FIELDCENTER_X2, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    static final TrcPose2D STARTPOS_BLUE_1                      =
        new TrcPose2D(-STARTPOS_FROM_FIELDCENTER_X1, STARTPOS_FROM_FIELDCENTER_Y, 180.0);
    static final TrcPose2D STARTPOS_BLUE_2                      =
        new TrcPose2D(STARTPOS_FROM_FIELDCENTER_X2, STARTPOS_FROM_FIELDCENTER_Y, 180.0);

    static final TrcPose2D RED_ALLIANCE_HUB_LOCATION            =
        new TrcPose2D(-HALF_TILE_INCHES, -FULL_TILE_INCHES, 0.0);
    static final TrcPose2D BLUE_ALLIANCE_HUB_LOCATION           =
        new TrcPose2D(-HALF_TILE_INCHES, FULL_TILE_INCHES, 180.0);
    static final TrcPose2D RED_CAROUSEL_LOCATION                =
        new TrcPose2D(-(HALF_FIELD_INCHES - 2.5), -(HALF_FIELD_INCHES - 2.5), 0.0);
    static final TrcPose2D BLUE_CAROUSEL_LOCATION               =
        new TrcPose2D(-(HALF_FIELD_INCHES - 2.5), HALF_FIELD_INCHES - 2.5, 180.0);
    static final TrcPose2D RED_STORAGE_UNIT_LOCATION            =
        new TrcPose2D(-(HALF_FIELD_INCHES - HALF_TILE_INCHES), -QUAD_FIELD_INCHES, 0.0);
    static final TrcPose2D BLUE_STORAGE_UNIT_LOCATION           =
        new TrcPose2D(-(HALF_FIELD_INCHES - HALF_TILE_INCHES), QUAD_FIELD_INCHES, 0.0);
    static final TrcPose2D SHARED_HUB_LOCATION                  =
        new TrcPose2D(FULL_TILE_INCHES*2.0, 0.0, 0.0);
    //
    // Motor Odometries.
    //
    static final double GOBILDA_5203_312_ENCODER_PPR            = ((((1.0 + (46.0/17.0)))*(1.0 + (46.0/11.0)))*28.0);
    static final double GOBILDA_5203_312_RPM                    = 312.0;
    static final double GOBILDA_5203_312_MAX_VELOCITY           =
        GOBILDA_5203_312_ENCODER_PPR*GOBILDA_5203_312_RPM/60.0; // encoder count per second.
    //
    // DriveBase subsystem.
    //
    static final DcMotor.RunMode DRIVE_MOTOR_MODE               = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final boolean LEFT_WHEEL_INVERTED                    = true;
    static final boolean RIGHT_WHEEL_INVERTED                   = false;
    static final boolean DRIVE_WHEEL_BRAKE_MODE                 = true;
//    static final double TURN_POWER_LIMIT                        = 0.5;
    static final double SLOW_DRIVE_POWER_SCALE                  = 0.5;
    static final double X_ODOMETRY_OFFSET                       = 8.0;  //8 inches in front of centroid
    //
    // Velocity controlled constants.
    //
    static final double DRIVE_WHEEL_GEAR_RATIO                  = 1.0;
    static final double DRIVE_WHEEL_DIAMETER                    = 4.0;  //inches
    static final double DRIVE_MOTOR_MAX_VELOCITY                = GOBILDA_5203_312_MAX_VELOCITY;    // unit: PPS

    static final double ENCODER_X_KP                            = 0.095;
    static final double ENCODER_X_KI                            = 0.0;
    static final double ENCODER_X_KD                            = 0.001;
    static final double ENCODER_X_TOLERANCE                     = 1.0;
    static final double ENCODER_X_INCHES_PER_COUNT              = 0.01924724265461924299065420560748;

    static final double ENCODER_Y_KP                            = 0.06;
    static final double ENCODER_Y_KI                            = 0.0;
    static final double ENCODER_Y_KD                            = 0.002;
    static final double ENCODER_Y_TOLERANCE                     = 1.0;
    static final double ENCODER_Y_INCHES_PER_COUNT              = 0.02166184604662450653409090909091;

    static final double GYRO_KP                                 = 0.009;
    static final double GYRO_KI                                 = 0.0;
    static final double GYRO_KD                                 = 0.0005;
    static final double GYRO_TOLERANCE                          = 2.0;

    static final double PIDDRIVE_STALL_TIMEOUT                  = 0.2;  //in seconds.
    //
    // Pure Pursuit parameters.
    //
    // goBILDA 5203-312 motor, max shaft speed = 312 RPM
    // motor-to-wheel gear ratio = 1:1
    // max wheel speed = pi * wheel diameter * motor RPM * wheel gear ratio / 60.0
    // = 3.1415926 * 4 in. * 312.0 / 60.0
    // = 65.345127 in./sec.
    static final double ROBOT_MAX_VELOCITY                      = Math.PI*DRIVE_WHEEL_DIAMETER*GOBILDA_5203_312_RPM/60.0;
    static final double ROBOT_MAX_ACCELERATION                  = 24.0;
    static final double ROBOT_VEL_KP                            = 0.0;
    static final double ROBOT_VEL_KI                            = 0.0;
    static final double ROBOT_VEL_KD                            = 0.0;
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    static final double ROBOT_VEL_KF                            = 1.0 / ROBOT_MAX_VELOCITY;
    //
    // Assuming the robot is placed at the center of the field for which we will set as field origin (i.e. x=0, y=0,
    // heading=0), this path will drive an infinity pattern.
    //
    static final TrcPose2D[] PURE_PURSUIT_PATH = new TrcPose2D[]{
        new TrcPose2D(-24.0, 0, 45.0),
        new TrcPose2D(-24.0, 48.0, 135.0),
        new TrcPose2D(24.0, 48.0, 225.0),
        new TrcPose2D(0.0, 46.0, 270.0),
        new TrcPose2D(0.0, 0.0, 0.0),

        new TrcPose2D(-23.0, 47.0, 225.0),
        new TrcPose2D(0.0, 0.0, 0.0)
    };


    //
    // Vision subsystem.
    //
    static final String TRACKABLE_IMAGES_FILE                   = "FreightFrenzy";
    static final double CAMERA_FRONT_OFFSET                     = 7.5;  //Camera offset from front of robot in inches
    static final double CAMERA_HEIGHT_OFFSET                    = 16.0; //Camera offset from floor in inches
    static final double CAMERA_LEFT_OFFSET                      = 8.875;//Camera offset from left of robot in inches

//    static final double HOMOGRAPHY_CAMERA_TOPLEFT_X             = 0.0;
//    static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y             = 360.0;
//    static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X            = 1280.0;
//    static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y            = 360.0;
//    static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X          = 0.0;
//    static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y          = 720.0;
//    static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X         = 1280.0;
//    static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y         = 720.0;
//    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
//    // Measurement unit: inches
//    static final double HOMOGRAPHY_WORLD_TOPLEFT_X              = -61.0;
//    static final double HOMOGRAPHY_WORLD_TOPLEFT_Y              = 83.0;
//    static final double HOMOGRAPHY_WORLD_TOPRIGHT_X             = 33.0;
//    static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y             = 83.0;
//    static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X           = -39.5;
//    static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y           = 19.0;
//    static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X          = 12.0;
//    static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y          = 19.0;
    //
    // Arm subsystem.
    //
    static final double ARM_KP                                  = 0.2;
    static final double ARM_KI                                  = 0.0;
    static final double ARM_KD                                  = 0.0;
    static final double ARM_TOLERANCE                           = 0.5;
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    static final double ARM_ENCODER_PPR                         = GOBILDA_5203_312_ENCODER_PPR;
    // https://www.gobilda.com/super-duty-worm-drive-pan-kit-28-1-ratio/
    static final double ARM_GEAR_RATIO                          = 28.0;
    static final double ARM_DEG_PER_COUNT                       = 360.0/(ARM_ENCODER_PPR*ARM_GEAR_RATIO);
    static final double ARM_OFFSET                              = 33.0;
    static final double ARM_MIN_POS                             = 33.0;
    static final double ARM_MAX_POS                             = 142.0;
    static final boolean ARM_MOTOR_INVERTED                     = true;
    static final boolean ARM_HAS_LOWER_LIMIT_SWITCH             = true;
    static final boolean ARM_LOWER_LIMIT_INVERTED               = false;
    static final boolean ARM_HAS_UPPER_LIMIT_SWITCH             = true;
    static final boolean ARM_UPPER_LIMIT_INVERTED               = false;
    static final double ARM_CAL_POWER                           = 0.5;
    static final double ARM_STALL_MIN_POWER                     = 0.3;
    static final double ARM_STALL_TIMEOUT                       = 1.0;
    static final double ARM_RESET_TIMEOUT                       = 0.5;
    static final double[] ARM_PRESET_LEVELS                     = new double[] {ARM_MIN_POS, 51.6, 76.2, 107};
    static final double ARM_SLOW_POWER_SCALE                    = 0.5;
    //
    // Intake subsystem.
    //
    static final double INTAKE_POWER_PICKUP                     = 1.0;
    static final double INTAKE_POWER_DUMP                       = -1.0;
    //
    // Spinner subsystem.
    //
    static final double SPINNER_POWER_RED                       = 1.0;
    static final double SPINNER_POWER_BLUE                      = -1.0;
    static final double SPINNER_TIME                            = 3.0;
    //
    // Odometry Wheel Deployer subsystem.
    //
    static final double ODWHEEL_YLEFT_RETRACT_POS               = 0.3;
    static final double ODWHEEL_YLEFT_EXTEND_POS                = 0.78;
    static final double ODWHEEL_YRIGHT_RETRACT_POS              = 0.3;
    static final double ODWHEEL_YRIGHT_EXTEND_POS               = 0.78;
    static final double ODWHEEL_X_RETRACT_POS                   = 0.3;
    static final double ODWHEEL_X_EXTEND_POS                    = 0.78;
    static final double ODWHEEL_X_INCHES_PER_COUNT              = 1.0;
    static final double ODWHEEL_Y_INCHES_PER_COUNT              = 1.0;

}   //class RobotInfo
