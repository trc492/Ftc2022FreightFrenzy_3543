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

import TrcCommonLib.command.CmdPurePursuitDrive;
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
        SPIN_CAROUSEL,
        DRIVE_TO_ALLIANCE_SHIPPING_HUB,
        DUMP_FREIGHT,
        DRIVE_TO_ALLIANCE_STORAGE_UNIT,
        DRIVE_TO_WAREHOUSE,

        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int duckPosition = 0;
//    private CmdPurePursuitDrive cmdPPDrive;

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
//        cmdPPDrive = new CmdPurePursuitDrive(
//                robot.robotDrive.driveBase, robot.robotDrive.xPosPidCoeff, robot.robotDrive.yPosPidCoeff,
//                robot.robotDrive.turnPidCoeff, robot.robotDrive.velPidCoeff);
        //should this remain commented?
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
        double yTarget = 0;
        double xTarget = 0;
        double degreeTarget = 0;
        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            boolean traceState = true;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            State nextState = null;
            switch (state)
            {

                case START_DELAY:
                    //
                    // Do start delay if any.
                    //
                    //call vision
                    if(robot.vision != null && robot.vision.isTensorFlowInitialized())
                    {
                        duckPosition = robot.vision.getLastDuckPosition();
                        robot.globalTracer.traceInfo(moduleName, "Duck found at position %d", duckPosition);
                    }
                    //if it cant find the duckPosition, set it to 2
                    if (duckPosition == 0) duckPosition = 2;


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
                    //if we are not doing carousel, skip to next state after spin carousel(drive to alliance shipping hub)
                    if(autoChoices.doCarousel== FtcAuto.Carousel.NO_CAROUSEL){
                        sm.setState(State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    }
                    else{
                        //based on alliance, drive to red or blue carousel
                        switch(autoChoices.alliance) {

                            case RED_ALLIANCE:
                                robot.robotDrive.purePursuitDrive.start(
                                        event,
                                        robot.robotDrive.driveBase.getFieldPosition(), false,
                                        robot.robotDrive.pathPoint(RobotParams.RED_CAROUSEL_CHECKPOINT_1, RobotParams.RED_CAROUSEL_CHECKPOINT_1_HEADING, true),
                                        robot.robotDrive.pathPoint(RobotParams.RED_CAROUSEL_LOCATION, 0.0, true));
                                break;
                            case BLUE_ALLIANCE:
                                robot.robotDrive.purePursuitDrive.start(
                                        event,
                                        robot.robotDrive.driveBase.getFieldPosition(), false,
                                        robot.robotDrive.pathPoint(RobotParams.BLUE_CAROUSEL_LOCATION, 0.0, true));
                                break;

                        }
                        //when done driving to carousel, go to next state - spinning carousel
                        sm.waitForSingleEvent(event, State.SPIN_CAROUSEL);

                        break;
                    }



                case SPIN_CAROUSEL:
                    //spins carousel with power depending on alliance color
                    robot.spinner.set(
                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                                    RobotParams.SPINNER_POWER_RED: RobotParams.SPINNER_POWER_BLUE,
                            RobotParams.SPINNER_TIME, event);
                    //once spinner is done go to next state - driving to shipping hub
                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    break;
                case DRIVE_TO_ALLIANCE_SHIPPING_HUB:
                    // sets arm to proper position based on duck position while driving
                    robot.arm.setPosition(RobotParams.ARM_PRESET_LEVELS[duckPosition]);

                    //if not driving to shipping hub and dumping freight go to next state after dumping freight - driving to storage unit
                        if(autoChoices.freightDelivery!=FtcAuto.FreightDelivery.DO_DELIVERY){
                            sm.setState(State.DRIVE_TO_ALLIANCE_STORAGE_UNIT);
                        }
                        else{
                            //based on alliance, drive to red or blue alliance hub
                            switch(autoChoices.alliance){
                                case RED_ALLIANCE :

                                    robot.robotDrive.purePursuitDrive.start(
                                            event,
                                            robot.robotDrive.driveBase.getFieldPosition(), false,
                                            robot.robotDrive.pathPoint(RobotParams.RED_ALLIANCE_HUB_CHECKPOINT_1, 0.0, true),
                                            robot.robotDrive.pathPoint(RobotParams.RED_ALLIANCE_HUB_LOCATION, 0.0, true));
                                    break;
                                case BLUE_ALLIANCE :
                                    robot.robotDrive.purePursuitDrive.start(
                                            event,
                                            robot.robotDrive.driveBase.getFieldPosition(), false,
                                            robot.robotDrive.pathPoint(RobotParams.BLUE_ALLIANCE_HUB_CHECKPOINT_1, 0.0, true),
                                            robot.robotDrive.pathPoint(RobotParams.BLUE_ALLIANCE_HUB_LOCATION, 0.0, true));
                                    break;
                            }
                            //when done driving to alliance hub, go to next state, dumping freight
                            sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                            break;
                        }

                //done
                case DUMP_FREIGHT:
                    //dumps the block for 2 seconds, when done signals event and goes to next state - driving to the storage unit
                    robot.intake.set(RobotParams.INTAKE_POWER_DUMP, 2, event);

                    sm.waitForSingleEvent(event, State. DRIVE_TO_ALLIANCE_STORAGE_UNIT);
                    break;
                case DRIVE_TO_ALLIANCE_STORAGE_UNIT:
                    //if not driving to the alliance storage unit, go to next state - driving to warehouse
                    if(autoChoices.parking!=FtcAuto.Parking.STORAGE_PARKING){
                        sm.setState(State.DRIVE_TO_WAREHOUSE);
                    }
                    else{
                        //based on alliance, drive to red or blue storage unit location with pure pursuit
                        switch(autoChoices.alliance){
                            case RED_ALLIANCE :
                                robot.robotDrive.purePursuitDrive.start(
                                        event,
                                        robot.robotDrive.driveBase.getFieldPosition(), false,
                                        robot.robotDrive.pathPoint(RobotParams.RED_STORAGE_UNIT_LOCATION, 0.0, true));
                                break;
                            case BLUE_ALLIANCE :
                                robot.robotDrive.purePursuitDrive.start(
                                        event,
                                        robot.robotDrive.driveBase.getFieldPosition(), false,
                                        robot.robotDrive.pathPoint(RobotParams.BLUE_STORAGE_UNIT_LOCATION, 0.0, true));
                                break;

                        }
                        //once done driving to the storage unit, go to next state - done
                        sm.waitForSingleEvent(event, State.DONE);

                        break;

                    }

                case DRIVE_TO_WAREHOUSE:
                    //if not parking at warehouse go to next state - done
                    if(autoChoices.parking!=FtcAuto.Parking.WAREHOUSE_PARKING){
                        sm.setState(State.DONE);
                    }
                    //based on alliance, drive to either the red or blue warehouse
                    switch(autoChoices.alliance){
                        case RED_ALLIANCE :
                            robot.robotDrive.purePursuitDrive.start(
                                    event,
                                    robot.robotDrive.driveBase.getFieldPosition(), false,

                                    robot.robotDrive.pathPoint(RobotParams.RED_WAREHOUSE_LOCATION_1, 0.0, true));
                            break;
                        case BLUE_ALLIANCE :
                            robot.robotDrive.purePursuitDrive.start(robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(RobotParams.BLUE_WAREHOUSE_LOCATION_1, 0.0, true));                                break;
                    }
                    // when done driving to warehouse, go to next state - done
                    sm.waitForSingleEvent(event, State.DONE);
                    break;





                case DONE:
                default:
                    //
                    // We are done.
                    //
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
