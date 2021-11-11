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

class CmdAutoFarCarousel implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoFarCarousel";

    private enum State
    {
        START_DELAY,
        DRIVE_TO_ALLIANCE_SHIPPING_HUB,
        DUMP_FREIGHT,
        DRIVE_TO_CAROUSEL,
        GET_TO_CAROUSEL,
        SPIN_CAROUSEL,
        DRIVE_TO_ALLIANCE_STORAGE_UNIT_FROM_CAROUSEL,
        DRIVE_TO_ALLIANCE_STORAGE_UNIT_FROM_NOT_CAROUSEL_INTERMEDIATE,
        DRIVE_TO_WAREHOUSE_FROM_NOT_CAROUSEL_INTERMEDIATE,
        DRIVE_TO_WAREHOUSE_FROM_CAROUSEL_INTERMEDIATE,
        RETRACT_ODOMETRY_WHEELS,
        DRIVE_TO_WAREHOUSE,

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
    CmdAutoFarCarousel(Robot robot, FtcAuto.AutoChoices autoChoices)
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
        double yTarget = 0.0;

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
                //done checking
                case START_DELAY:
                    robot.robotDrive.driveBase.setFieldPosition(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ?
                            RobotParams.STARTPOS_RED_2 : RobotParams.STARTPOS_BLUE_2);
                    //
                    // Do start delay if any.
                    //
                    // Call vision at the beginning to figure out the position of the duck.
                    if (robot.vision != null && robot.vision.isTensorFlowInitialized())
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
                    }

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

                    //done
                case DRIVE_TO_ALLIANCE_SHIPPING_HUB:
                    // If not driving to shipping hub and dumping freight, go to next state (driving to carousel).
                    if (autoChoices.freightDelivery == FtcAuto.FreightDelivery.NO_DELIVERY)
                    {
                        sm.setState(State.DRIVE_TO_CAROUSEL);
                    }
                    else
                    {
                        // Drive to alliance shipping hub.
                        // Note: the smaller the number the closer to the hub.
                        double distanceToHub = duckPosition == 3 ? 0.4 : duckPosition == 2 ? 0.4 : 0.4;
                        //only difference between coordinate for red and blue case is the yTarget
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                //heading is negative because we are dumping freight to the right of shipping hub
                                robot.robotDrive.pathPoint(distanceToHub, -1.0, -90.0, true));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                //heading is negative because we are dumping freight to the right of shipping hub
                                robot.robotDrive.pathPoint(distanceToHub, 1.0, -90.0, true));
                        }

                        // Raise arm to the detected duck level at the same time.
                        robot.arm.setLevel(duckPosition);
                        sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                    }
                    break;
                //done
                case DUMP_FREIGHT:
                    // Dumps the freight for 2 seconds, when done signals event and goes to next state (driving to the
                    // storage unit).
                    robot.intake.set(RobotParams.INTAKE_POWER_DUMP, 2.0, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_CAROUSEL);
                    break;
                //done
                case DRIVE_TO_CAROUSEL:
                    // If we are not doing carousel, skip to next state (drive to storage unit from not carousel)
                    if (autoChoices.doCarousel == FtcAuto.Carousel.NO_CAROUSEL)
                    {
                        sm.setState(State.DRIVE_TO_ALLIANCE_STORAGE_UNIT_FROM_NOT_CAROUSEL_INTERMEDIATE);
                    }
                    else
                    {
                        // Drive to the carousel.
                        // assuming that we wont need to use PidDrive for drive to Carousel
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(0.5, 0.0, -90.0, true),
                                robot.robotDrive.pathPoint(-2.0, 0.0, -90.0, true),
                                robot.robotDrive.pathPoint(-2.5, -2.2, 0.0, true));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                //first two points same as for red alliance, for last point, yTarget and heading is different
                                robot.robotDrive.pathPoint(0.5, 0.0, -90.0, true),
                                robot.robotDrive.pathPoint(-2.0, 0.0, -90.0, true),
                                robot.robotDrive.pathPoint(-2.5, 2.2, 180.0, true));
                        }
                        sm.waitForSingleEvent(event, State.GET_TO_CAROUSEL);
                    }
                    break;
                //done
                case GET_TO_CAROUSEL:
                    // We are still about an inch from the carousel, drive slowly towards it for 200 msec to touch it.
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.2, 0.0, false);
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.SPIN_CAROUSEL);
                    break;

                //done
                case SPIN_CAROUSEL:
                    // We touched the carousel, so stop the drive base.
                    robot.robotDrive.driveBase.stop();
                    // Spin the carousel for 3 seconds.
                    robot.spinner.set(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ?
                            RobotParams.SPINNER_POWER_RED : RobotParams.SPINNER_POWER_BLUE,
                        RobotParams.SPINNER_TIME, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_STORAGE_UNIT_FROM_CAROUSEL);
                    break;

                //DONE
                case DRIVE_TO_ALLIANCE_STORAGE_UNIT_FROM_CAROUSEL:
                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        // We are not parking anywhere, just stop and be done.
                        sm.setState(State.DONE);
                    }
                    else if (autoChoices.parking == FtcAuto.Parking.WAREHOUSE_PARKING)
                    {
                        // We are parking at the warehouse.
                        sm.setState(State.DRIVE_TO_WAREHOUSE_FROM_CAROUSEL_INTERMEDIATE);
                    }
                    else
                    {
                        // Drive to alliance storage unit
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, -1.5, 0.0, true));
                        }
                        else
                        {
                            //pure pursuit alternative
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, 1.5, 180.0, true));
                        }
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;
                //done
                case DRIVE_TO_ALLIANCE_STORAGE_UNIT_FROM_NOT_CAROUSEL_INTERMEDIATE:
                    //for both red and blue the coordinate an dheading is the same
                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        sm.setState(State.DONE);
                    }
                    else if (autoChoices.parking == FtcAuto.Parking.WAREHOUSE_PARKING)
                    {
                        sm.setState(State.DRIVE_TO_WAREHOUSE_FROM_NOT_CAROUSEL_INTERMEDIATE);
                    }
                    else
                    {
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            // You may be coming from start position or alliance shipping hub.
                            robot.robotDrive.purePursuitDrive.start(
                                event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(0.5, 0.0, -90.0),
                                robot.robotDrive.pathPoint(-2.0, 0.0, -90.0),
                                robot.robotDrive.pathPoint(-2.5, -1.5, 0.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(0.5, 0.0, -90.0),
                                robot.robotDrive.pathPoint(-2.0, 0.0, -90.0),
                                robot.robotDrive.pathPoint(-2.5, 1.5, 180.0));
                        }
                        sm.waitForSingleEvent(event, State.DONE);
                        break;
                    }

                case DRIVE_TO_WAREHOUSE_FROM_CAROUSEL_INTERMEDIATE:
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-2.0, 0.0, 0.0),
                            robot.robotDrive.pathPoint(0.5, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, -1.8, 90.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-2.0, 0.0, 0.0),
                            robot.robotDrive.pathPoint(0.5, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, 1.8, 90.0));
                    }
                    sm.waitForSingleEvent(event, State.DRIVE_TO_WAREHOUSE);
                    break;

                case DRIVE_TO_WAREHOUSE_FROM_NOT_CAROUSEL_INTERMEDIATE:
                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        // You may be coming from either start position or alliance shipping hub.
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(0.5, -1.8, -90.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(0.5, 1.8, -90.0));
                        }
                        sm.waitForSingleEvent(event, State.RETRACT_ODOMETRY_WHEELS);
                    }
                    break;

                case RETRACT_ODOMETRY_WHEELS:
                    robot.arm.setLevel(1);
                    robot.odwDeployer.retract();
                    timer.set(2.0, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_WAREHOUSE);
                    break;

                case DRIVE_TO_WAREHOUSE:
                    double heading = robot.robotDrive.driveBase.getHeading();
                    robot.robotDrive.driveBase.holonomicDrive(0.0, heading < 0.0 ? -1.0 : 1.0, 0.0);
                    timer.set(0.5, event);
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

}   //class CmdAutoFarCarousel
