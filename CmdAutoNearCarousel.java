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
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;

class CmdAutoNearCarousel implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoNearCarousel";

    private enum State
    {
        START_DELAY,
        DRIVE_TO_CAROUSEL,
        GET_TO_CAROUSEL,
        SPIN_CAROUSEL,
        DRIVE_TO_ALLIANCE_SHIPPING_HUB,
        DUMP_FREIGHT,
        DRIVE_TO_ALLIANCE_STORAGE_UNIT,
        DRIVE_TO_WAREHOUSE,
        RETRACT_ODOMETRY_WHEELS,
        GET_INTO_WAREHOUSE,

        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int duckPosition = 0;

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

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case START_DELAY:
                    //
                    // Do start delay if any.
                    //
                    robot.robotDrive.driveBase.setFieldPosition(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            RobotParams.STARTPOS_RED_NEAR : RobotParams.STARTPOS_BLUE_NEAR);
                    // Call vision at the beginning to figure out the position of the duck.
                    if(robot.vision != null && robot.vision.isTensorFlowInitialized())
                    {
                        duckPosition = robot.vision.getLastDuckPosition();
                        robot.globalTracer.traceInfo(moduleName, "Duck found at position %d", duckPosition);
                    }

                    if (duckPosition == 0)
                    {
                        //
                        // We still can't see the duck, default to level 3.
                        //
                        duckPosition = 3;
                        robot.globalTracer.traceInfo(
                            moduleName, "No duck found, default to position %d.", duckPosition);
                    }

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
                    if (autoChoices.doCarousel == FtcAuto.Carousel.NO_CAROUSEL)
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
                                event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, -2.2, 0.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, 2.2, 180.0));
                        }
                        sm.waitForSingleEvent(event, State.GET_TO_CAROUSEL);
                    }
                    break;

                case GET_TO_CAROUSEL:
                    // We are still about an inch from the carousel, drive slowly towards it for 400 msec to touch it.
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.2, 0.0, false);
                    timer.set(
                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.4: 0.5, event);
                    sm.waitForSingleEvent(event, State.SPIN_CAROUSEL);
                    break;

                case SPIN_CAROUSEL:
                    // We touched the carousel, so stop the drive base.
                    robot.robotDrive.driveBase.stop();
                    // Spin the carousel for 3 seconds.
                    robot.spinner.set(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            RobotParams.SPINNER_POWER_RED: RobotParams.SPINNER_POWER_BLUE,
                        RobotParams.SPINNER_TIME, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    break;

                case DRIVE_TO_ALLIANCE_SHIPPING_HUB:
                    if (autoChoices.freightDelivery == FtcAuto.FreightDelivery.NO_DELIVERY)
                    {
                        // We are not doing freight delivery, go to next state.
                        sm.setState(State.DRIVE_TO_ALLIANCE_STORAGE_UNIT);
                    }
                    else
                    {
                        // Drive to alliance shipping hub. We could be coming from starting position or carousel.
                        // Note: the smaller the number the closer to the hub.
                        double distanceToHub = duckPosition == 3? 1.3: duckPosition == 2? 1.5: 1.4;

                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, -2.0, 0.0),
                                robot.robotDrive.pathPoint(-2.5, -0.9, 0.0),
                                robot.robotDrive.pathPoint(-distanceToHub, -0.9, 90.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, 2.0, 180.0),
                                robot.robotDrive.pathPoint(-2.5, 0.9, 180.0),
                                robot.robotDrive.pathPoint(-distanceToHub, 0.9, 90.0));
                        }
                        // Raise arm to the detected duck level at the same time.
                        robot.arm.setLevel(duckPosition);
                        sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                    }
                    break;

                case DUMP_FREIGHT:
                    // Dumps the freight for 2 seconds, when done signals event and goes to next state
                    robot.intake.set(RobotParams.INTAKE_POWER_DUMP, 2.0, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_STORAGE_UNIT);
                    break;

                case DRIVE_TO_ALLIANCE_STORAGE_UNIT:
                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        // We are not parking anywhere, just stop and be done.
                        sm.setState(State.DONE);
                    }
                    else if (autoChoices.parking == FtcAuto.Parking.WAREHOUSE_PARKING)
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
                                event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, -1.4, 90.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, 1.4,90.0));
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
                            event, 10.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            RobotParams.ROBOT_MAX_VELOCITY, RobotParams.ROBOT_MAX_ACCELERATION,
                            robot.robotDrive.pathPoint(-2.5, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, -1.6, 90.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 10.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-2.5, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, 1.6, 90.0));
                    }
                    sm.waitForSingleEvent(event, State.RETRACT_ODOMETRY_WHEELS);
                    break;

                case RETRACT_ODOMETRY_WHEELS:
                    // We are going over barriers, so retract everything to their proper heights and wait 2 seconds
                    // for them to be done.
                    robot.arm.setLevel(1);
                    robot.odwDeployer.retract();
                    timer.set(2.0, event);
                    sm.waitForSingleEvent(event, State.GET_INTO_WAREHOUSE);
                    break;

                case GET_INTO_WAREHOUSE:
                    // Run full speed into the warehouse crossing the barriers.
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 1.0, 0.0);
                    timer.set(0.75, event);
                    sm.waitForSingleEvent(event, State.DONE);
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
                robot.traceStateInfo(sm.getState());
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoNearCarousel