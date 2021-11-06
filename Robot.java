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
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcRevBlinkin;
import TrcFtcLib.ftclib.FtcRobotBattery;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcMotorActuator;
import TrcFtcLib.ftclib.FtcServo;
import TrcFtcLib.ftclib.FtcServoActuator;

import java.util.Locale;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    //
    // Global objects.
    //
    private static final String ROBOT_NAME = "Robot3543";
    private static final String OPENCV_NATIVE_LIBRARY_NAME = "opencv_java3";
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
    public FtcRevBlinkin blinkin;
    public FtcRobotBattery battery;
    //
    // Subsystems.
    //
    public RobotDrive robotDrive = null;
    public FtcMotorActuator arm = null;
    public FtcDcMotor intake = null;
    public FtcDcMotor spinner = null;
    public OdometryWheelDeployer odwDeployer = null;
    public FtcServoActuator pickupHook = null;

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
        // Vision may use blinkin, so we must instantiate it before vision. However, if we are doing vision only, we
        // don't have robot hardware thus no blinkin either.
        //
        if (RobotParams.Preferences.useBlinkin && !RobotParams.Preferences.visionOnly)
        {
            blinkin = new FtcRevBlinkin(RobotParams.HWNAME_BLINKIN);
        }
        //
        // Initialize vision subsystems.
        //
        if (RobotParams.Preferences.useVuforia || RobotParams.Preferences.useTensorFlow)
        {
            vision = new Vision(this);

            if (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE)
            {
                if (RobotParams.Preferences.useVuforia)
                {
                    vision.initVuforia();
                }

                if (RobotParams.Preferences.useTensorFlow)
                {
                    System.loadLibrary(OPENCV_NATIVE_LIBRARY_NAME);
                    vision.initTensorFlow();
                }

                if (RobotParams.Preferences.useBlinkin && blinkin != null)
                {
                    vision.setupBlinkin();
                }
            }
        }
        //
        // If visionOnly is true, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        //
        if (!RobotParams.Preferences.visionOnly)
        {
            //
            // Create and initialize sensors and indicators.
            //
            if (RobotParams.Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }
            //
            // Create and initialize RobotDrive.
            //
            robotDrive = new RobotDrive();
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.initSubsystems)
            {
                if (RobotParams.Preferences.useArm)
                {
                    final FtcMotorActuator.Parameters armParams = new FtcMotorActuator.Parameters()
                        .setPosRange(RobotParams.ARM_MIN_POS, RobotParams.ARM_MAX_POS)
                        .setScaleOffset(RobotParams.ARM_DEG_PER_COUNT, RobotParams.ARM_OFFSET)
                        .setPidParams(RobotParams.ARM_KP, RobotParams.ARM_KI, RobotParams.ARM_KD, RobotParams.ARM_TOLERANCE)
                        .setMotorParams(
                            RobotParams.ARM_MOTOR_INVERTED,
                            RobotParams.ARM_HAS_LOWER_LIMIT_SWITCH, RobotParams.ARM_LOWER_LIMIT_INVERTED,
                            RobotParams.ARM_HAS_UPPER_LIMIT_SWITCH, RobotParams.ARM_UPPER_LIMIT_INVERTED,
                            RobotParams.ARM_CAL_POWER)
                        .setStallProtectionParams(
                            RobotParams.ARM_STALL_MIN_POWER, RobotParams.ARM_STALL_TIMEOUT, RobotParams.ARM_RESET_TIMEOUT)
                        .setPosPresets(RobotParams.ARM_PRESET_LEVELS);
                    arm = new FtcMotorActuator(RobotParams.HWNAME_ARM, armParams);
                    //
                    // Don't zero calibrate the arm. Some tests may want the arm up a bit.
                    //
                    if (runMode != TrcRobot.RunMode.TEST_MODE)
                    {
                        arm.zeroCalibrate();
                    }
                }
                intake = new FtcDcMotor(RobotParams.HWNAME_INTAKE);
                spinner = new FtcDcMotor(RobotParams.HWNAME_SPINNER);
                odwDeployer = new OdometryWheelDeployer();
                //
                // Deploy ODW only if we are using external odometry and we are in either autonomous or test modes.
                //
                if (RobotParams.Preferences.useExternalOdometry &&
                    (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
                {
                    odwDeployer.deploy();
                }
                else
                {
                    odwDeployer.retract();
                }
                FtcServoActuator.Parameters pickupHookParams = new FtcServoActuator.Parameters()
                    .setStepParams(
                        RobotParams.PICKUPHOOK_STEPRATE, RobotParams.PICKUPHOOK_MIN_POS, RobotParams.PICKUPHOOK_MAX_POS)
                    .setInverted(false, false)
                    .setRetractParams(RobotParams.PICKUPHOOK_RETRACT_POS, RobotParams.PICKUPHOOK_ROTATE_TIME)
                    .setExtendParams(RobotParams.PICKUPHOOK_EXTEND_POS, RobotParams.PICKUPHOOK_ROTATE_TIME);
                pickupHook = new FtcServoActuator(RobotParams.HWNAME_PICKUP_HOOK, pickupHookParams);
                pickupHook.retract();
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
        final String funcName = "startMode";

        if (arm != null)
        {
            // Raise the arm a little at start so it will not get caught on the floor tile.
            arm.setPosition(RobotParams.ARM_MIN_POS + 5.0);
        }

        if (robotDrive != null)
        {
            //
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);
            }
            //
            // Enable odometry only for autonomous or test modes.
            //
            if (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE)
            {
                robotDrive.setOdometryEnabled(true);
            }
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
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.setElapsedTimerEnabled(true);
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
        final String funcName = "stopMode";
        //
        // Print all performance counters if there are any.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.printElapsedTime(globalTracer);
            robotDrive.gyro.setElapsedTimerEnabled(false);
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

        if (robotDrive != null)
        {
            //
            // Disable odometry.
            //
            robotDrive.setOdometryEnabled(false);
            //
            // Disable gyro task.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    }   //stopMode

    /**
     * This method is typically called in the autonomous state machine to log the autonomous state info as a state
     * event in the trace log file. The logged event can be used to play back autonomous path movement.
     *
     * @param state specifies the current state of the state machine.
     */
    public void traceStateInfo(Object state)
    {
        final String funcName = "traceStateInfo";

        // TODO: traceState does not support PurePursuitDrive yet.
        if (robotDrive != null)
        {
            StringBuilder msg = new StringBuilder();

            msg.append(String.format(Locale.US, "tag=\">>>>>\" state=\"%s\"", state));
            if (robotDrive.pidDrive.isActive())
            {
                TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();
                TrcPose2D targetPose = robotDrive.pidDrive.getAbsoluteTargetPose();

                if (robotDrive.encoderXPidCtrl != null)
                {
                    msg.append(String.format(Locale.US, " xPos=%6.2f xTarget=%6.2f", robotPose.x, targetPose.x));
                }

                if (robotDrive.encoderYPidCtrl != null)
                {
                    msg.append(String.format(Locale.US, " yPos=%6.2f yTarget=%6.2f", robotPose.y, targetPose.y));
                }

                if (robotDrive.gyroPidCtrl != null)
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
