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

import android.os.Environment;

import com.qualcomm.robotcore.hardware.DcMotor;

import TrcCommonLib.trclib.TrcPose2D;

public class RobotParams
{
    public static class Preferences
    {
        static boolean noRobot = false;
        static boolean initSubsystems = true;
        static boolean useExternalOdometry = true;
        static boolean useArm = true;
        static boolean useIntake = true;
        static boolean hasIntakeSensor = true;
        static boolean usePickupHook = false;
        static boolean useBlinkin = true;
        static boolean useVuforia = false;
        static boolean showVuforiaView = false;
        static boolean useTensorFlow = true;
        static boolean showTensorFlowView = true;
        static boolean useTraceLog = true;
        static boolean useBatteryMonitor = true;
        static boolean playSongs = false;
        static boolean useLoopPerformanceMonitor = true;
        static boolean useVelocityControl = false;
    }   //class Preferences

    public enum DriveMode
    {
        TANK_MODE,
        HOLONOMIC_MODE,
        TIM_MODE
    }   //enum DriveMode

    static final String LOG_PATH_FOLDER                         =
        Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc3543";
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
    static final String HWNAME_INTAKE                           = "intake";
    static final String HWNAME_SPINNER                          = "spinnerMotor";
    static final String HWNAME_YLEFT_ODW_DEPLOYOR               = "yLeftOdwServo";
    static final String HWNAME_YRIGHT_ODW_DEPLOYOR              = "yRightOdwServo";
    static final String HWNAME_X_ODW_DEPLOYOR                   = "xOdwServo";
    static final String HWNAME_PICKUP_HOOK                      = "pickupHookServo";
    //
    // Field dimensions.
    //
    static final double FULL_FIELD_INCHES                       = 141.0;
    static final double HALF_FIELD_INCHES                       = FULL_FIELD_INCHES/2.0;
    static final double QUAD_FIELD_INCHES                       = FULL_FIELD_INCHES/4.0;
    static final double FULL_TILE_INCHES                        = 23.75;
    static final double HALF_TILE_INCHES                        = FULL_TILE_INCHES/2.0;
    //
    // Robot dimensions.
    //
    static final double ROBOT_LENGTH                            = 17.0;
    static final double ROBOT_WIDTH                             = 13.0;
    //
    // Game positions.
    //
    static final double STARTPOS_FROM_FIELDCENTER_Y             = HALF_FIELD_INCHES - ROBOT_LENGTH/2.0;
    static final double STARTPOS_FROM_FIELDCENTER_X1            = QUAD_FIELD_INCHES;
    static final double STARTPOS_FROM_FIELDCENTER_X2            = HALF_TILE_INCHES;

    static final TrcPose2D STARTPOS_RED_NEAR                    =
        new TrcPose2D(-STARTPOS_FROM_FIELDCENTER_X1, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    static final TrcPose2D STARTPOS_RED_FAR                     =
        new TrcPose2D(STARTPOS_FROM_FIELDCENTER_X2, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    static final TrcPose2D STARTPOS_BLUE_NEAR                   =
        new TrcPose2D(-STARTPOS_FROM_FIELDCENTER_X1, STARTPOS_FROM_FIELDCENTER_Y, 180.0);
    static final TrcPose2D STARTPOS_BLUE_FAR                    =
        new TrcPose2D(STARTPOS_FROM_FIELDCENTER_X2, STARTPOS_FROM_FIELDCENTER_Y, 180.0);
    //
    // Motor Odometries.
    //
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    static final double GOBILDA_5203_312_ENCODER_PPR            = ((((1.0 + (46.0/17.0)))*(1.0 + (46.0/11.0)))*28.0);
    static final double GOBILDA_5203_312_RPM                    = 312.0;
    static final double GOBILDA_5203_312_MAX_VELOCITY_PPS       =
        GOBILDA_5203_312_ENCODER_PPR*GOBILDA_5203_312_RPM/60.0; // 2795.987 pps
    //
    // DriveBase subsystem.
    //
    static final DriveMode ROBOT_DRIVE_MODE                     = DriveMode.TIM_MODE;
    static final DcMotor.RunMode DRIVE_MOTOR_MODE               = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final boolean LEFT_WHEEL_INVERTED                    = true;
    static final boolean RIGHT_WHEEL_INVERTED                   = false;
    static final boolean DRIVE_WHEEL_BRAKE_MODE                 = true;
    static final double TURN_POWER_LIMIT                        = 0.5;
    static final double SLOW_DRIVE_POWER_SCALE                  = 0.5;
    static final double X_ODOMETRY_WHEEL_OFFSET                 = ROBOT_LENGTH/2.0 - (3.875 + 9.5); //behind centroid
    static final double Y_LEFT_ODOMETRY_WHEEL_OFFSET            = -15.25/2.0;
    static final double Y_RIGHT_ODOMETRY_WHEEL_OFFSET           = 15.25/2.0;
    //
    // Velocity controlled constants.
    //
    static final double DRIVE_MOTOR_MAX_VELOCITY_PPS            = GOBILDA_5203_312_MAX_VELOCITY_PPS;

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

    static final double GYRO_KP                                 = 0.023;
    static final double GYRO_KI                                 = 0.0;
    static final double GYRO_KD                                 = 0.001;
    static final double GYRO_TOLERANCE                          = 2.0;

    static final double ODWHEEL_X_INCHES_PER_COUNT              = 7.6150160901199168116026724971383e-4;
    static final double ODWHEEL_Y_INCHES_PER_COUNT              = 7.6301149255006038191364659148717e-4;

    static final double PIDDRIVE_STALL_TIMEOUT                  = 0.2;  //in seconds.
    //
    // Pure Pursuit parameters.
    //
    // No-Load max velocity (i.e. theoretical maximum)
    // goBILDA 5203-312 motor, max shaft speed = 312 RPM
    // motor-to-wheel gear ratio = 1:1
    // max wheel speed = pi * wheel diameter * wheel gear ratio * motor RPM / 60.0
    // = 3.1415926535897932384626433832795 * 4 in. * 1.0 * 312.0 / 60.0
    // = 65.345127194667699360022982372214 in./sec.
    static final double ROBOT_MAX_VELOCITY                      = 25.0;     // measured maximum from drive speed test.
    static final double ROBOT_MAX_ACCELERATION                  = 3380.0;   // measured maximum from drive speed test.
    static final double ROBOT_VEL_KP                            = 0.0;
    static final double ROBOT_VEL_KI                            = 0.0;
    static final double ROBOT_VEL_KD                            = 0.0;
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    static final double ROBOT_VEL_KF                            = 1.0 / ROBOT_MAX_VELOCITY;
    static final double PPD_FOLLOWING_DISTANCE                  = 6.0;
    static final double PPD_POS_TOLERANCE                       = 2.0;
    static final double PPD_TURN_TOLERANCE                      = 2.0;

    static final double HOMOGRAPHY_CAMERA_TOPLEFT_X             = 0.0;
    static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y             = 0.0;
    static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X            = 639.0;
    static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y            = 0.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X          = 0.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y          = 479.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X         = 639.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y         = 479.0;

    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
    // Measurement unit: inches
    static final double HOMOGRAPHY_WORLD_TOPLEFT_X              = -22.25;
    static final double HOMOGRAPHY_WORLD_TOPLEFT_Y              = 60.0;
    static final double HOMOGRAPHY_WORLD_TOPRIGHT_X             = 23.0;
    static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y             = 60.0;
    static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X           = -8.75;
    static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y           = 16.0;
    static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X          = 7.5;
    static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y          = 16.0;
    //
    // Vision subsystem.
    //
    static final String TRACKABLE_IMAGES_FILE                   = "FreightFrenzy";
    static final double CAMERA_FRONT_OFFSET                     = 7.5;  //Camera offset from front of robot in inches
    static final double CAMERA_HEIGHT_OFFSET                    = 16.0; //Camera offset from floor in inches
    static final double CAMERA_LEFT_OFFSET                      = 6.0;  //Camera offset from left of robot in inches
    //
    // Arm subsystem.
    //
    static final double ARM_KP                                  = 0.2;
    static final double ARM_KI                                  = 0.0;
    static final double ARM_KD                                  = 0.0;
    static final double ARM_TOLERANCE                           = 0.5;
    static final double ARM_ENCODER_PPR                         = GOBILDA_5203_312_ENCODER_PPR;
    // https://www.gobilda.com/super-duty-worm-drive-pan-kit-28-1-ratio/
    static final double ARM_GEAR_RATIO                          = 28.0;
    static final double ARM_DEG_PER_COUNT                       = 360.0/(ARM_ENCODER_PPR*ARM_GEAR_RATIO);
    static final double ARM_OFFSET                              = 33.0;
    static final double ARM_MIN_POS                             = 32.0;
    static final double ARM_MAX_POS                             = 135.0;
    static final double ARM_TRAVEL_POS                          = ARM_MIN_POS + 5.0;
    static final boolean ARM_MOTOR_INVERTED                     = true;
    static final boolean ARM_HAS_LOWER_LIMIT_SWITCH             = true;
    static final boolean ARM_LOWER_LIMIT_INVERTED               = false;
    static final boolean ARM_HAS_UPPER_LIMIT_SWITCH             = true;
    static final boolean ARM_UPPER_LIMIT_INVERTED               = false;
    static final double ARM_CAL_POWER                           = 0.5;
    static final double ARM_STALL_MIN_POWER                     = 0.3;
    static final double ARM_STALL_TIMEOUT                       = 1.0;
    static final double ARM_RESET_TIMEOUT                       = 0.5;
    static final double[] ARM_PRESET_LEVELS                     = new double[] {ARM_MIN_POS, 51.6, 78, 107};
    static final double ARM_SLOW_POWER_SCALE                    = 0.5;
    //
    // Intake subsystem.
    //
    static final double INTAKE_POWER_PICKUP                     = 1.0;
    static final double INTAKE_POWER_DUMP                       = -0.6;
    static final double INTAKE_DUMP_TIME                        = 1.2;
    static final double INTAKE_SENSOR_THRESHOLD                 = 4.0;    //in cm
    //
    // Spinner subsystem.
    //
    static final double SPINNER_POWER_RED                       = 1.0;
    static final double SPINNER_POWER_BLUE                      = -1.0;
    static final double SPINNER_TIME                            = 2.0;
    //
    // Odometry Wheel Deployer subsystem.
    //
    static final double ODWHEEL_YLEFT_RETRACT_POS               = 0.25;
    static final double ODWHEEL_YLEFT_EXTEND_POS                = 0.78;
    static final double ODWHEEL_YRIGHT_RETRACT_POS              = 0.25;
    static final double ODWHEEL_YRIGHT_EXTEND_POS               = 0.78;
    static final double ODWHEEL_X_RETRACT_POS                   = 0.25;
    static final double ODWHEEL_X_EXTEND_POS                    = 0.78;
    //
    // Pickup Hook subsystem.
    //
    static final double PICKUPHOOK_MIN_POS                      = 0.0;
    static final double PICKUPHOOK_MAX_POS                      = 1.0;
    static final double PICKUPHOOK_RETRACT_POS                  = PICKUPHOOK_MIN_POS;
    static final double PICKUPHOOK_EXTEND_POS                   = PICKUPHOOK_MAX_POS;
    static final double PICKUPHOOK_STEPRATE                     = (1.0/0.75);
    static final double PICKUPHOOK_ROTATE_TIME                  = 0.3;

}   //class RobotInfo
