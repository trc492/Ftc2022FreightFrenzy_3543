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

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcTensorFlow;

class CmdAutoNearCarousel implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoNearCarousel";
    private static final double PARK_WAREHOUSE_TIME = 8.0;

    private enum State
    {
        START_DELAY,
        DRIVE_TO_CAROUSEL,
        GET_TO_CAROUSEL,
        SPIN_CAROUSEL,
        DRIVE_TO_ALLIANCE_SHIPPING_HUB,
        DUMP_FREIGHT,

        POSITION_TO_FIND_THE_DUCK,
        FIND_THE_DUCK,
        GO_PICKUP_DUCK,
        DONE_PICKUP_DUCK,

        DRIVE_TO_ALLIANCE_STORAGE_UNIT,
        DRIVE_TO_WAREHOUSE,
        RETRACT_ODOMETRY_WHEELS,
        GET_INTO_WAREHOUSE,
        GET_TO_WAREHOUSE_CENTER,

        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int duckPosition = 0;
    private FtcTensorFlow.TargetInfo targetInfo = null;
    private Double expireTime = null;
    private boolean deliveringDuck = false;
    private int retryCount = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAutoNearCarousel(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        robot.robotDrive.purePursuitDrive.setFastModeEnabled(true);
        sm.start(State.START_DELAY);
    }   //CmdAutoNearCarousel

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
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            RobotParams.STARTPOS_RED_NEAR : RobotParams.STARTPOS_BLUE_NEAR);
                    // Call vision at the beginning to figure out the position of the duck.
                    if (robot.vision != null)
                    {
                        duckPosition = robot.vision.getLastDuckPosition();
                        msg = "Duck found at position " + duckPosition;
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }

                    if (duckPosition == 0)
                    {
                        //
                        // We still can't see the duck, default to level 3.
                        //
                        duckPosition = 3;
                        msg = "No duck found, default to position " + duckPosition;
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }
                    robot.arm.setLevel(1);
                    //
                    // Do start delay if any.
                    //
                    if (autoChoices.startDelay == 0.0)
                    {
                        //
                        // Intentionally falling through to the next state.
                        //
                        sm.setState(State.DRIVE_TO_CAROUSEL);
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_CAROUSEL);
                        break;
                    }

                case DRIVE_TO_CAROUSEL:
                    if (!autoChoices.doCarousel)
                    {
                        // We are not doing carousel, skip to next state.
                        sm.setState(State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    }
                    else
                    {
                        // Drive to the carousel from the starting position.
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, -2.1, 0.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, 2.1, 180.0));
                        }
                        sm.waitForSingleEvent(event, State.GET_TO_CAROUSEL);
                    }
                    break;

                case GET_TO_CAROUSEL:
                    // We are a few inches from the carousel, drive slowly towards it to touch it.
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.2, 0.0, false);
                    timer.set(autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.8: 0.8, event);
                    sm.waitForSingleEvent(event, State.SPIN_CAROUSEL);
                    break;

                case SPIN_CAROUSEL:
                    // We touched the carousel, so stop the drive base.
                    robot.robotDrive.driveBase.stop();
                    // Spin the carousel.
                    robot.spinner.set(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            RobotParams.SPINNER_POWER_RED: RobotParams.SPINNER_POWER_BLUE,
                        RobotParams.SPINNER_TIME + 1.0, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    break;

                case DRIVE_TO_ALLIANCE_SHIPPING_HUB:
                    if (!autoChoices.freightDelivery)
                    {
                        // We are not doing freight delivery, go to next state.
                        sm.setState(State.POSITION_TO_FIND_THE_DUCK);
                    }
                    else
                    {
                        // Drive to alliance shipping hub. We could be coming from starting position or carousel.
                        // Note: the smaller the number the closer to the hub.
                        double distanceToHub;
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                             distanceToHub = duckPosition == 3? 1.1: duckPosition == 2? 1.3: 1.3;
                        }
                        else
                        {
                            distanceToHub = duckPosition == 3? 1.1: duckPosition == 2? 1.3: 1.3;

                        }

                        if (deliveringDuck)
                        {
                            // We are coming from the duck pickup point somewhere near the carousel.
                            if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                            {
                                robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-2.0, -1.0, 90.0),
                                    robot.robotDrive.pathPoint(-distanceToHub, -1.0, 90.0));
                            }
                            else
                            {
                                robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-2.0, 1.0, 90.0),
                                    robot.robotDrive.pathPoint(-distanceToHub, 1.0, 90.0));
                            }
                        }
                        // We are delivering freight, so we could be coming from starting position or the carousel.
                        else if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, -2.0, 0.0),
                                robot.robotDrive.pathPoint(-2.5, -0.9, 0.0),
                                robot.robotDrive.pathPoint(-2.0, -1.0, 90.0),
                                robot.robotDrive.pathPoint(-distanceToHub, -1.0, 90.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, 2.0, 180.0),
                                robot.robotDrive.pathPoint(-2.5, 0.9, 180.0),
                                robot.robotDrive.pathPoint(-2.0, 1.0, 90.0),
                                robot.robotDrive.pathPoint(-distanceToHub, 1.0, 90.0));
                        }
                        // Raise arm to the detected duck level at the same time.
                        robot.arm.setLevel(duckPosition);
                        // After we dump the freight to the right level for the bonus, any subsequent dumps will be to
                        // the top.
                        duckPosition = 3;
                        sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                    }
                    break;

                case DUMP_FREIGHT:
                    // Dumps the freight, when done, signals event and goes to next state.
                    robot.intake.setPower(RobotParams.INTAKE_POWER_DUMP, RobotParams.INTAKE_DUMP_TIME, event);
                    sm.waitForSingleEvent(
                        event, deliveringDuck? State.DRIVE_TO_ALLIANCE_STORAGE_UNIT: State.POSITION_TO_FIND_THE_DUCK);
                    break;

                case POSITION_TO_FIND_THE_DUCK:
                    robot.arm.setLevel(0.25, 0);
                    // If we did not do Carousel, there is no duck on the floor, so skip it.
                    if (!autoChoices.duckDelivery || !autoChoices.doCarousel)
                    {
                        sm.setState(State.DRIVE_TO_ALLIANCE_STORAGE_UNIT);
                    }
                    else
                    {
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.0, -1.7, 180.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.0, 1.7, 0.0));
                        }
                        retryCount = 0;
                        sm.waitForSingleEvent(event, State.FIND_THE_DUCK);
                    }
                    break;

                case FIND_THE_DUCK:
                    targetInfo = robot.vision.getDetectedDuckInfo();
                    if (targetInfo != null)
                    {
                        if (robot.blinkin != null)
                        {
                            robot.blinkin.setPatternState(Vision.sawTarget, true);
                        }
                        robot.globalTracer.traceInfo(
                            moduleName, "Found the duck at x=%.1f, y=%.1f, angle=%.1f",
                            targetInfo.distanceFromCamera.x, targetInfo.distanceFromCamera.y, targetInfo.angle);
                        robot.speak("Found the duck.");
                        sm.setState(State.GO_PICKUP_DUCK);
                    }
                    else if (expireTime == null)
                    {
                        expireTime = TrcUtil.getCurrentTime() + 2.0;
                    }
                    else if (TrcUtil.getCurrentTime() > expireTime)
                   {
                       if (retryCount == 0)
                       {
                           // Did not find the duck the first time, zoom in and look again.
                           robot.globalTracer.traceInfo(moduleName, "<<<<< Zoom in  and look again!");
                           robot.speak("Zoom in and Look again ");
                           robot.vision.setTensorFlowZoomFactor(1.5);
                           sm.setState(State.FIND_THE_DUCK);
                           expireTime = null;
                           retryCount++;
                       }
                       else if (retryCount == 1)
                       {
                           // Did not find the duck the second time, move to the side and look again.
                           robot.globalTracer.traceInfo(moduleName, "<<<<< Move to the side and look again!");
                           robot.speak("Move to the side and look again.");
                           if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                           {
                               robot.robotDrive.purePursuitDrive.start(
                                   event, robot.robotDrive.driveBase.getFieldPosition(), true,
                                   new TrcPose2D(-10.0, 0.0, 0.0));
                           }
                           else
                           {
                               robot.robotDrive.purePursuitDrive.start(
                                   event, robot.robotDrive.driveBase.getFieldPosition(), true,
                                   new TrcPose2D(10.0, 0.0, 0.0));
                           }
                           sm.waitForSingleEvent(event, State.FIND_THE_DUCK);
                           expireTime = null;
                           retryCount++;
                       }
                       else
                       {
                           robot.globalTracer.traceInfo(moduleName, "<<<<< Duck not found, giving up!");
                           robot.speak("Duck not found, giving up.");
                           // Did not find the duck the third time, give up and park.
                           sm.setState(State.DRIVE_TO_ALLIANCE_STORAGE_UNIT);
                       }
                    }
                    break;

                case GO_PICKUP_DUCK:
                    // Call pure pursuit using robot field position +
                    // after tuning distanceFromCenter will be real world distance from robot where robot is (0, 0)
                    // turns such that the robot is inline with the game piece and moves forward
                    // assumes distance from game object gives distance from computer to actual object
                    //robot.intake.autoAssist(RobotParams.INTAKE_POWER_PICKUP, event2, null, 3.0);
                    robot.intake.setPower(RobotParams.INTAKE_POWER_PICKUP);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.25);
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(
                            targetInfo.distanceFromCamera.x,
                            targetInfo.distanceFromCamera.y - 6.0,    //adjust for intake offset from robot centroid.
                            targetInfo.angle));
                    deliveringDuck = true;
                    sm.waitForSingleEvent(event, State.DONE_PICKUP_DUCK);
                    break;

                case DONE_PICKUP_DUCK:
                    // Keep spinning the intake at low power to keep the duck from falling out.
                    robot.intake.setPower(0.3);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                    sm.setState(State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    break;

                case DRIVE_TO_ALLIANCE_STORAGE_UNIT:
                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        // We are not parking anywhere, just stop and be done.
                        sm.setState(State.DONE);
                    }
                    // If we don't have enough time to go to the warehouse, park at the storage unit instead.
                    else if (autoChoices.parking == FtcAuto.Parking.WAREHOUSE_PARKING  &&
                             30.0 - elapsedTime > PARK_WAREHOUSE_TIME)
                    {
                        // We are parking at the warehouse.
                        sm.setState(State.DRIVE_TO_WAREHOUSE);
                    }
                    else
                    {
                        // Drive to storage unit. We could be coming from starting position, carousel or alliance hub.
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, -1.4, 90.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, 1.4, 90.0));
                        }
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case DRIVE_TO_WAREHOUSE:
                    // We are heading to the warehouse but we could be coming from starting position, carousel or
                    // alliance hub.
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-2.0, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, -1.6, 90.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-2.0, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, 1.6, 90.0));
                    }
                    sm.waitForSingleEvent(event, State.RETRACT_ODOMETRY_WHEELS);
                    break;

                case RETRACT_ODOMETRY_WHEELS:
                    // We are going over barriers, so retract everything to their proper heights and wait 2 seconds
                    // for them to be done.
                    // After retracting od wheels, check the time before running into warehouse in case our alliance
                    // partner is still cycling.
                    robot.arm.setLevel(1);
                    robot.odwDeployer.retract();
                    timer.set(30.0 - elapsedTime - 3.0, event);
                    sm.waitForSingleEvent(event, State.GET_INTO_WAREHOUSE);
                    break;

                case GET_INTO_WAREHOUSE:
                    // Run full speed into the warehouse crossing the barriers.
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 1.0, 0.0);
                    timer.set(0.9, event);
                    sm.waitForSingleEvent(event, State.GET_TO_WAREHOUSE_CENTER);
                    break;

                case GET_TO_WAREHOUSE_CENTER:
                    robot.robotDrive.driveBase.holonomicDrive(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE? 1.0: -1.0, 0.0, 0.0);
                    timer.set(0.25, event);
                    sm.waitForSingleEvent(event, State.DONE);
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
                robot.globalTracer.traceStateInfo(
                    sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoNearCarousel