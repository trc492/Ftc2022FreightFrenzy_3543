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
        DRIVE_TO_WAREHOUSE_INTERMEDIATE,
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
    CmdAutoNearCarousel(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        robot.robotDrive.driveBase.setFieldPosition(
            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                RobotParams.STARTPOS_RED_1: RobotParams.STARTPOS_BLUE_1);
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
                    // Call vision at the beginning to figure out the position of the duck.
                    if(robot.vision != null && robot.vision.isTensorFlowInitialized())
                    {
                        robot.vision.getCurrentDuckPositions();
                        duckPosition = robot.vision.getLastDuckPosition();
                        robot.globalTracer.traceInfo(moduleName, "Duck found at position %d", duckPosition);
                    }

                    // If it can't find the duckPosition, set it to 2 as default.
                    if (duckPosition == 0 && elapsedTime < 10.0)
                    {
                        //
                        // We can't find the duck. Keep looking for up to 10 second.
                        //
                        break;
                    }

                    if (duckPosition == 0)
                    {
                        //
                        // We still can't see the duck, default to level 2.
                        //
                        duckPosition = 2;
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
                        //when delay is over, goes to next state - drive to Carousel
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_CAROUSEL);
                        break;
                    }

                case DRIVE_TO_CAROUSEL:
                    // If we are not doing carousel, skip to next state after spin carousel (drive to alliance
                    // shipping hub)
                    if (autoChoices.doCarousel == FtcAuto.Carousel.NO_CAROUSEL)
                    {
                        sm.setState(State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    }
                    else
                    {
                        // Based on alliance, drive to red or blue carousel
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, -2.0, 0.0, true),
                                robot.robotDrive.pathPoint(-2.5, -2.3, 0.0, true));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, 2.0, 180.0, true),
                                robot.robotDrive.pathPoint(-2.5, 2.3, 180.0, true));
                        }
                        //when done driving to carousel, go to next state - spinning carousel
                        sm.waitForSingleEvent(event, State.GET_TO_CAROUSEL);
                    }
                    break;

                case GET_TO_CAROUSEL:
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.2, 0.0, false);
                    timer.set(0.2, event);
                    sm.waitForSingleEvent(event, State.SPIN_CAROUSEL);
                    break;

                case SPIN_CAROUSEL:
                    robot.robotDrive.driveBase.stop();
                    //spins carousel with power depending on alliance color
                    robot.spinner.set(
                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                                    RobotParams.SPINNER_POWER_RED: RobotParams.SPINNER_POWER_BLUE,
                            RobotParams.SPINNER_TIME, event);
                    //once spinner is done go to next state - driving to shipping hub
                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    break;

                case DRIVE_TO_ALLIANCE_SHIPPING_HUB:
                    //if not driving to shipping hub and dumping freight go to next state after dumping freight - driving to storage unit
                    if (autoChoices.freightDelivery == FtcAuto.FreightDelivery.NO_DELIVERY)
                    {
                        sm.setState(State.DRIVE_TO_ALLIANCE_STORAGE_UNIT);
                    }
                    else
                    {
                        // based on alliance, drive to red or blue alliance hub
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.0, -1.0, 90.0, true),
                                robot.robotDrive.pathPoint(-1.3, -1.0, 90.0, true));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.0, 1.0, 90.0, true),
                                robot.robotDrive.pathPoint(-1.3, 1.0, 90.0, true));
                        }
                        // raise arm to the detected duck level at the same time.
                        robot.arm.setLevel(duckPosition);
                        //when done driving to alliance hub, go to next state, dumping freight
                        sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                    }
                    break;

                case DUMP_FREIGHT:
                    //dumps the freight for 2 seconds, when done signals event and goes to next state - driving to the
                    // storage unit
                    robot.intake.set(RobotParams.INTAKE_POWER_DUMP, 2.0, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_STORAGE_UNIT);
                    break;

                case DRIVE_TO_ALLIANCE_STORAGE_UNIT:
                    //if not driving to the alliance storage unit, go to next state - driving to warehouse
                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        sm.setState(State.DONE);
                    }
                    else if (autoChoices.parking == FtcAuto.Parking.WAREHOUSE_PARKING)
                    {
                        sm.setState(State.DRIVE_TO_WAREHOUSE_INTERMEDIATE);
                    }
                    else
                    {
                        //based on alliance, drive to red or blue storage unit location with pure pursuit
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, -1.5, 0.0, true));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, 1.5,180.0, true));
                        }
                        //once done driving to the storage unit, go to next state - done
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case DRIVE_TO_WAREHOUSE_INTERMEDIATE:
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-1.5, -1.5, 90.0, true));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-1.5, 1.5, 90.0, true));
                    }
                    sm.waitForSingleEvent(event, State.DRIVE_TO_WAREHOUSE);
                    break;

                case DRIVE_TO_WAREHOUSE:
                    //set arm to level 1 before crossing obstacles into the warehouse.
                    robot.arm.setLevel(1);
                    //based on alliance, drive to either the red or blue warehouse
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(2.5, -1.5, 90.0, true));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(2.5, 1.5, 90.0, true));
                    }
                    // when done driving to warehouse, go to next state - done
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
