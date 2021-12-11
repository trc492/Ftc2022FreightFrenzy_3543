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

import java.util.Locale;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;

class CmdAutoShuttleBackAndForth implements TrcRobot.RobotCommand
{
    static final double FAST_ENCODER_X_KP                            = 0.1;
    static final double FAST_ENCODER_X_KI                            = 0.0;
    static final double FAST_ENCODER_X_KD                            = 0.0095;

    static final double FAST_ENCODER_Y_KP                            = 0.04;
    static final double FAST_ENCODER_Y_KI                            = 0.0;
    static final double FAST_ENCODER_Y_KD                            = 0.004;

    private static final String moduleName = "CmdAutoShuttleBackAndForth";
    private static final double ROUND_TRIP_TIME = 8.0;
    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int duckPosition = 0;

    private enum State
    {
        START_DELAY,
        DRIVE_TO_ALLIANCE_SHIPPING_HUB,
        DUMP_FREIGHT,
        DRIVE_INTO_WAREHOUSE,
        PICK_UP_FREIGHT_FROM_WAREHOUSE,
        DETERMINE_ROUND_TRIP_OR_DONE,
        DRIVE_OUT_OF_WAREHOUSE_TO_SHIPPING_HUB,

        DONE
    }   //enum State

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAutoShuttleBackAndForth(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);

//        robot.robotDrive.purePursuitDrive.setXPositionPidCoefficients(new TrcPidController.PidCoefficients(
//                FAST_ENCODER_X_KP, FAST_ENCODER_X_KI, FAST_ENCODER_X_KD));
//
//       robot.robotDrive.purePursuitDrive.setYPositionPidCoefficients(new TrcPidController.PidCoefficients(FAST_ENCODER_Y_KP, FAST_ENCODER_Y_KI, FAST_ENCODER_Y_KD));
//
//
//       robot.robotDrive.purePursuitDrive.setFastModeEnabled(true);

        sm.start(State.START_DELAY);
    }   //CmdAutoShuttleBackAndForth

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        robot.robotDrive.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        //
        // Assume narrow robot with width less than 14 inches.
        //
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            boolean traceState = true;
            String msg;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case START_DELAY:
                    //
                    // Set robot starting position in the field.
                    //
                    robot.robotDrive.driveBase.setFieldPosition(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ?
                            RobotParams.STARTPOS_RED_FAR : RobotParams.STARTPOS_BLUE_FAR);
                    // Call vision at the beginning to figure out the position of the duck.
                    if (robot.vision != null)
                    {
                        duckPosition = robot.vision.getLastDuckPosition();
                        msg = String.format(Locale.US, "Duck found at position %d.", duckPosition);
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }

                    if (duckPosition == 0)
                    {
                        //
                        // We still can't see the duck, default to level 3.
                        //
                        duckPosition = 3;
                        msg = String.format(Locale.US, "No duck found, default to position %d.", duckPosition);
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }
                    //
                    // Do start delay if any.
                    //
                    if (autoChoices.startDelay == 0.0)
                    {
                        //
                        // Intentionally falling through to the next state.
                        //
                        sm.setState(State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                        break;
                    }

                case DRIVE_TO_ALLIANCE_SHIPPING_HUB:
                    // Drive to alliance shipping hub.
                    // Note: the smaller the number the closer to the hub.
                    double distanceToHub;

                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        distanceToHub = duckPosition == 3? 1.87: duckPosition == 2? 1.95: 1.95;
                    }
                    else
                    {
                        distanceToHub = duckPosition == 3? 1.9: duckPosition == 2? 1.95: 1.9;
                    }

                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-0.5, -distanceToHub, 0.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-0.5, distanceToHub, 180.0));
                    }
                    // Raise arm to the detected duck level at the same time.
                    robot.arm.setLevel(duckPosition);
                    //after we dump the duck to the right level for the bonus, any subsequent dumps will be to the top
                    duckPosition = 3;
                    sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                    break;

                case DUMP_FREIGHT:
                    // Dumps the freight, when done signals event and goes to next state
                    //robot.intake.autoAssist(RobotParams.INTAKE_POWER_DUMP, event, null, RobotParams.INTAKE_DUMP_TIME);

                    robot.intake.setPower(-0.5, RobotParams.INTAKE_DUMP_TIME, event);
                    sm.waitForSingleEvent(event, State.DRIVE_INTO_WAREHOUSE);
                    break;

                case DRIVE_INTO_WAREHOUSE:
                    // Fire and forget with lowering the arm.
                    //robot.arm.setTarget(0.3, RobotParams.ARM_MIN_POS, false, null, 1.0);
                    robot.arm.setTarget(RobotParams.ARM_TRAVEL_POS);
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            //RobotParams.ROBOT_MAX_VELOCITY, RobotParams.ROBOT_MAX_ACCELERATION,
                            robot.robotDrive.pathPoint(-0.5, -2, 0),
                            robot.robotDrive.pathPoint(0.5, -2.55, 90.0),
                            robot.robotDrive.pathPoint(1.6, -2.55, 90.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            //RobotParams.ROBOT_MAX_VELOCITY, RobotParams.ROBOT_MAX_ACCELERATION,
                                robot.robotDrive.pathPoint(-0.5, 2, 0),
                                robot.robotDrive.pathPoint(0.5, 2.55, 90.0),
                                robot.robotDrive.pathPoint(1.6, 2.55, 90.0));
                    }
                    sm.waitForSingleEvent(event, State.PICK_UP_FREIGHT_FROM_WAREHOUSE);
                    break;

                case PICK_UP_FREIGHT_FROM_WAREHOUSE:
                    // If there are only 10 seconds left in autonomous, we go to done because we are already in the
                    // warehouse timeout is timeleft-roundtriptime
                    robot.globalTracer.traceInfo(moduleName, "arm position=%.1f", robot.arm.getPosition());
                    robot.intake.autoAssist(
                        RobotParams.INTAKE_POWER_PICKUP, event, null, 30.0 - elapsedTime - ROUND_TRIP_TIME);
                    //keep running drive base until next event is signaled
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.25);

                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            null, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(2.6, -2.7, 90.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            null, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(2.6, 2.7, 90.0));
                    }
                    //event is signaled by intake when robot picked up a block or timeout expired
                    sm.waitForSingleEvent(event, State.DETERMINE_ROUND_TRIP_OR_DONE);
                    break;

                case DETERMINE_ROUND_TRIP_OR_DONE:
                    // If intake has freight and there are more than round trip time left
                    robot.robotDrive.purePursuitDrive.cancel();
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                    if (robot.intake.hasObject() && 30.0 - elapsedTime > ROUND_TRIP_TIME)
                    {
                        //if it has freight, keep running intake so block doesnt fall out
                        robot.intake.setPower(RobotParams.INTAKE_POWER_PICKUP);
                        //next state is driving out of warehouse (to try to dump the block)
                        sm.setState(State.DRIVE_OUT_OF_WAREHOUSE_TO_SHIPPING_HUB);
                    }
                    else
                    {
                        //turn off intake, set state to done
                        sm.setState(State.DONE);
                    }
                    break;

                case DRIVE_OUT_OF_WAREHOUSE_TO_SHIPPING_HUB:
                    distanceToHub = 1.8;
                    robot.arm.setLevel(3);
                    //back out to around the place where we start but still facing the warehouse and then drive to the shipping hub
                    if (autoChoices.alliance==FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 8.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            //RobotParams.ROBOT_MAX_VELOCITY, RobotParams.ROBOT_MAX_ACCELERATION,
                            robot.robotDrive.pathPoint(0.2, -2.7,  90.0),
                            robot.robotDrive.pathPoint(-0.5, -distanceToHub, 0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 8.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            //RobotParams.ROBOT_MAX_VELOCITY, RobotParams.ROBOT_MAX_ACCELERATION,

                            robot.robotDrive.pathPoint(0.2, 2.7,  90.0),
                            robot.robotDrive.pathPoint(-0.5, distanceToHub, 180));
                    }
                    //next state is dumping
                    sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                    break;

                case DONE:
                default:
                    //
                    // We are done, zero calibrate the arm will lower it.
                    //
                    robot.arm.zeroCalibrate();
                    cancel();
                    break;
            }

            if (traceState)
            {
                robot.traceStateInfo(state);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoShuttleBackAndForth
