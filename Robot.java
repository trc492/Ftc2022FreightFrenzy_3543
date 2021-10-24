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

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcDriveBaseOdometry;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcMecanumDriveBase;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcFtcLib.ftclib.FtcBNO055Imu;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcRevBlinkin;
import TrcFtcLib.ftclib.FtcRobotBattery;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcMotorActuator;
import TrcFtcLib.ftclib.FtcServo;

import java.util.Locale;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    //
    // Preferences and parameters.
    //
    public static class Preferences
    {
        static boolean visionOnly = false;
        static boolean initSubsystems = true;
        static boolean useExternalOdometry = false;
        static boolean hasArm = false;
        static boolean hasBlinkin = false;
        static boolean useVuforia = false;
        static boolean showVuforiaView = false;
        static boolean useTensorFlow = true;
        static boolean showTensorFlowView = true;
        static boolean useBlinkinFlashLight = false;
        static boolean useTraceLog = true;
        static boolean useBatteryMonitor = true;
        static boolean useLoopPerformanceMonitor = true;
        static boolean useVelocityControl = false;
    }   //class Preferences

    public enum DriveMode
    {
        TANK_MODE,
        HOLONOMIC_MODE,
    }   //enum DriveMode

    //
    // Global objects.
    //
    private static final String OPENCV_NATIVE_LIBRARY_NAME = "opencv_java3";
    private static final String ROBOT_NAME = "Ftc3543";
    public FtcOpMode opMode;
    public FtcDashboard dashboard;
    public TrcDbgTrace globalTracer;
    //
    // Vision subsystems.
    //
    public Vision vision;
    //
    // Sensors and indicators.
    //
    public FtcRobotBattery battery;
    public FtcRevBlinkin blinkin;
    public FtcBNO055Imu imu;
    public TrcGyro gyro;
    //
    // DriveBase subsystem.
    //
    public FtcDcMotor leftFrontWheel = null;
    public FtcDcMotor rightFrontWheel = null;
    public FtcDcMotor leftBackWheel = null;
    public FtcDcMotor rightBackWheel = null;

    public TrcDriveBase driveBase = null;
    public DriveMode driveMode = DriveMode.HOLONOMIC_MODE;
    public TrcPidController encoderXPidCtrl = null;
    public TrcPidController encoderYPidCtrl = null;
    public TrcPidController gyroPidCtrl = null;
    public TrcPidDrive pidDrive = null;

    // Pure Pursuit PID controllers.
    public TrcPidController.PidCoefficients xPosPidCoeff = null;
    public TrcPidController.PidCoefficients yPosPidCoeff = null;
    public TrcPidController.PidCoefficients turnPidCoeff = null;
    public TrcPidController.PidCoefficients velPidCoeff = null;

    public TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients();
    //
    // Subsystems.
    //
    public FtcMotorActuator arm = null;
    public FtcServo wrist = null;
    public FtcDcMotor intake = null;
    public FtcDcMotor spinner = null;
    public FtcServo odometryWheelDeployer = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *                specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTextView(
            ((FtcRobotControllerActivity)opMode.hardwareMap.appContext)
                .findViewById(com.qualcomm.ftcrobotcontroller.R.id.textOpMode));
        globalTracer = TrcDbgTrace.getGlobalTracer();
        //
        // Initialize vision subsystems.
        //
        if (Preferences.useVuforia || Preferences.useTensorFlow)
        {
            vision = new Vision(this);

            if (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE)
            {
                if (Preferences.useVuforia)
                {
                    vision.initVuforia();
                }

                if (Preferences.useTensorFlow)
                {
                    vision.initTensorFlow();
                }
            }
        }
        //
        // If visionOnly is true, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        //
        if (!Preferences.visionOnly)
        {
            //
            // Initialize sensors and indicators.
            //
            if (Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }

            if (Preferences.hasBlinkin)
            {
                blinkin = new FtcRevBlinkin("blinkin");
            }
            imu = new FtcBNO055Imu(RobotInfo.IMU_NAME);
            gyro = imu.gyro;
            //
            // Initialize DriveBase.
            //
            initDriveBase();
            //
            // Initialize other subsystems.
            //
            if (Preferences.initSubsystems)
            {
                if (Preferences.hasArm)
                {
                    final FtcMotorActuator.Parameters armParams = new FtcMotorActuator.Parameters()
                        .setPosRange(RobotInfo.ARM_MIN_HEIGHT, RobotInfo.ARM_MAX_HEIGHT)
                        .setScaleOffset(RobotInfo.ARM_SCALE, RobotInfo.ARM_OFFSET)
                        .setPidParams(
                            RobotInfo.ARM_KP, RobotInfo.ARM_KI, RobotInfo.ARM_KD,
                            RobotInfo.ARM_TOLERANCE)
                        .setMotorParams(
                            RobotInfo.ARM_INVERTED, RobotInfo.ARM_HAS_LOWER_LIMIT_SWITCH,
                            RobotInfo.ARM_HAS_UPPER_LIMIT_SWITCH, RobotInfo.ARM_CAL_POWER)
                        .setStallProtectionParams(
                            RobotInfo.ARM_STALL_MIN_POWER, RobotInfo.ARM_STALL_TIMEOUT,
                            RobotInfo.ARM_RESET_TIMEOUT);
                    arm = new FtcMotorActuator(RobotInfo.ARM_NAME, armParams);
                    arm.zeroCalibrate();
                }
                intake = new FtcDcMotor(RobotInfo.INTAKE_NAME);
                spinner = new FtcDcMotor(RobotInfo.SPINNER_NAME);
            }
        }
    }   //Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return ROBOT_NAME;
    }   //toString

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "Robot.startMode";
        //
        // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
        //
        if (gyro != null)
        {
            gyro.setEnabled(true);
        }
        //
        // Enable odometry only for autonomous or test modes.
        //
        if (driveBase != null && (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            leftFrontWheel.setOdometryEnabled(true);
            rightFrontWheel.setOdometryEnabled(true);
            leftBackWheel.setOdometryEnabled(true);
            rightBackWheel.setOdometryEnabled(true);
            driveBase.setOdometryEnabled(true);
        }
        //
        // Vision generally will impact performance, so we only enable it if it's needed such as in autonomous.
        //
        if (vision != null && (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            if (vision.isVuforiaInitialized())
            {
                globalTracer.traceInfo(funcName, "Enabling Vuforia.");
                vision.setVuforiaEnabled(true);
            }

            if (vision.isTensorFlowInitialized())
            {
                globalTracer.traceInfo(funcName, "Enabling TensorFlow.");
                vision.setTensorFlowEnabled(true);
            }
        }
        //
        // The following are performance counters, could be disabled for competition if you want.
        // But it might give you some insight if somehow autonomous wasn't performing as expected.
        //
        if (gyro != null)
        {
            gyro.setElapsedTimerEnabled(true);
        }
        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "Robot.stopMode";
        //
        // Print all performance counters if there are any.
        //
        if (gyro != null)
        {
            gyro.printElapsedTime(globalTracer);
            gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);
        //
        // Disable vision.
        //
        if (vision != null)
        {
            if (vision.isVuforiaInitialized())
            {
                globalTracer.traceInfo(funcName, "Disabling Vuforia.");
                vision.setVuforiaEnabled(false);
            }

            if (vision.isTensorFlowInitialized())
            {
                globalTracer.traceInfo(funcName, "Shutting down TensorFlow.");
                vision.tensorFlowShutdown();
            }
        }
        //
        // Disable odometry.
        //
        if (driveBase != null)
        {
            driveBase.setOdometryEnabled(false);
            leftFrontWheel.setOdometryEnabled(false);
            rightFrontWheel.setOdometryEnabled(false);
            leftBackWheel.setOdometryEnabled(false);
            rightBackWheel.setOdometryEnabled(false);
        }
        //
        // Disable gyro task.
        //
        if (gyro != null)
        {
            gyro.setEnabled(false);
        }
    }   //stopMode

    /**
     * This method turns the flashlight ON or OFF for the vision subsystem.
     *
     * @param blinkinFlashOn specifies true to turn on the Blinkin LED, false to turn off.
     */
    public void setFlashLightOn(boolean blinkinFlashOn)
    {
        if (blinkin != null && Preferences.useBlinkinFlashLight)
        {
            blinkin.setPattern(
                blinkinFlashOn? TrcRevBlinkin.LEDPattern.SolidWhite: TrcRevBlinkin.LEDPattern.SolidBlack);
        }
    }   //setFlashLightOn

    /**
     * This method creates and initializes the drive base related components.
     */
    private void initDriveBase()
    {
        leftFrontWheel = new FtcDcMotor(RobotInfo.LEFT_FRONT_WHEEL_NAME);
        rightFrontWheel = new FtcDcMotor(RobotInfo.RIGHT_FRONT_WHEEL_NAME);
        leftBackWheel = new FtcDcMotor(RobotInfo.LEFT_BACK_WHEEL_NAME);
        rightBackWheel = new FtcDcMotor(RobotInfo.RIGHT_BACK_WHEEL_NAME);

        leftFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        leftBackWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightBackWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);

        if (Preferences.useVelocityControl)
        {
            leftFrontWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY);
            rightFrontWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY);
            leftBackWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY);
            rightBackWheel.enableVelocityMode(RobotInfo.MOTOR_MAX_VELOCITY);
        }

        leftFrontWheel.setInverted(RobotInfo.LEFT_WHEEL_INVERTED);
        leftBackWheel.setInverted(RobotInfo.LEFT_WHEEL_INVERTED);
        rightFrontWheel.setInverted(RobotInfo.RIGHT_WHEEL_INVERTED);
        rightBackWheel.setInverted(RobotInfo.RIGHT_WHEEL_INVERTED);

        leftFrontWheel.setBrakeModeEnabled(RobotInfo.DRIVE_WHEEL_BRAKE_MODE);
        leftBackWheel.setBrakeModeEnabled(RobotInfo.DRIVE_WHEEL_BRAKE_MODE);
        rightFrontWheel.setBrakeModeEnabled(RobotInfo.DRIVE_WHEEL_BRAKE_MODE);
        rightBackWheel.setBrakeModeEnabled(RobotInfo.DRIVE_WHEEL_BRAKE_MODE);

        driveBase = new TrcMecanumDriveBase(leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, gyro);
        if (Preferences.useExternalOdometry)
        {
            //
            // Create the external odometry device that uses the left front encoder port as the X odometry and
            // the left and right back encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
            // odometry.
            //
            TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
                new TrcDriveBaseOdometry.AxisSensor(leftFrontWheel, RobotInfo.X_ODOMETRY_OFFSET),
                new TrcDriveBaseOdometry.AxisSensor[] {
                    new TrcDriveBaseOdometry.AxisSensor(leftBackWheel),
                    new TrcDriveBaseOdometry.AxisSensor(rightBackWheel)},
                gyro);
            //
            // Set the drive base to use the external odometry device overriding the built-in one.
            //
            driveBase.setDriveBaseOdometry(driveBaseOdometry);
        }
        driveBase.setOdometryScales(RobotInfo.ENCODER_X_INCHES_PER_COUNT, RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        driveMode = DriveMode.HOLONOMIC_MODE;
        //
        // Initialize PID drive.
        //
        xPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD);
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD);
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotInfo.ROBOT_VEL_KP, RobotInfo.ROBOT_VEL_KI, RobotInfo.ROBOT_VEL_KD, RobotInfo.ROBOT_VEL_KF);

        encoderXPidCtrl = new TrcPidController(
            "encoderXPidCtrl", xPosPidCoeff, RobotInfo.ENCODER_X_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl", yPosPidCoeff, RobotInfo.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
            "gyroPidCtrl", turnPidCoeff, RobotInfo.GYRO_TOLERANCE, driveBase::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputLimit(RobotInfo.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setMsgTracer(globalTracer);
    }   //initDriveBase

    /**
     * This method is typically called in the autonomous state machine to log the autonomous state info as a state
     * event in the trace log file. The logged event can be used to play back autonomous path movement.
     *
     * @param state specifies the current state of the state machine.
     */
    public void traceStateInfo(Object state)
    {
        final String funcName = "traceStateInfo";

        if (driveBase != null)
        {
            StringBuilder msg = new StringBuilder();

            msg.append(String.format(Locale.US, "tag=\">>>>>\" state=\"%s\"", state));
            if (pidDrive.isActive())
            {
                TrcPose2D robotPose = driveBase.getFieldPosition();
                TrcPose2D targetPose = pidDrive.getAbsoluteTargetPose();

                if (encoderXPidCtrl != null)
                {
                    msg.append(String.format(Locale.US, " xPos=%6.2f xTarget=%6.2f", robotPose.x, targetPose.x));
                }

                if (encoderYPidCtrl != null)
                {
                    msg.append(String.format(Locale.US, " yPos=%6.2f yTarget=%6.2f", robotPose.y, targetPose.y));
                }

                if (gyroPidCtrl != null)
                {
                    msg.append(String.format(Locale.US, " heading=%6.2f headingTarget=%6.2f",
                                             robotPose.angle, targetPose.angle));
                }
            }

            if (battery != null)
            {
                msg.append(String.format(Locale.US,
                                         " volt=\"%5.2fV(%5.2fV)\"", battery.getVoltage(), battery.getLowestVoltage()));
            }

            globalTracer.logEvent(funcName, "StateInfo", "%s", msg);
        }
    }   //traceStateInfo

}   //class Robot
