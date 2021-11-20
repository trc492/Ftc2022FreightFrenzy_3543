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
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcTensorFlow;

class CmdAutoShuttleBackAndForth implements TrcRobot.RobotCommand
{

    private static final String moduleName = "CmdAutoShuttleBackAndForth";
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
        DRIVE_OUT_OF_WAREHOUSE,


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
        //assumes are robot is less than 14 inches
        //
        State state = sm.checkReadyAndGetState();
        boolean grabberHasPickedBlock = false;
        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            boolean traceState = true;
            String msg;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            Double  expireTime = null;
            FtcTensorFlow.TargetInfo targetInfo = null;
            switch (state)
            {
                case START_DELAY:
                    //
                    // Do start delay if any.
                    //
                    robot.robotDrive.driveBase.setFieldPosition(
                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
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
                        // Drive to alliance shipping hub. We could be coming from starting position or carousel.
                        // Note: the smaller the number the closer to the hub.
                        if(elapsedTime>=20){
                            sm.setState(State.DRIVE_INTO_WAREHOUSE);
                        }
                        double distanceToHub = duckPosition == 3? 1.3: duckPosition == 2? 1.45: 1.4;

                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                    event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-2.5, -2.0, 0.0),
                                    robot.robotDrive.pathPoint(-2.5, -0.9, 0.0),
                                    robot.robotDrive.pathPoint(-0.5, -distanceToHub, 90.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                    event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-2.5, 2.0, 180.0),
                                    robot.robotDrive.pathPoint(-2.5, 0.9, 180.0),
                                    robot.robotDrive.pathPoint(-0.5, distanceToHub, 90.0));
                        }
                        // Raise arm to the detected duck level at the same time.

                        robot.arm.setLevel(duckPosition);
                        //after we dump the duck to the right level for the bonus, any subsequent dumps will be to the top
                        duckPosition = 3;
                        sm.waitForSingleEvent(event, State.DUMP_FREIGHT);

                        break;

                case DUMP_FREIGHT:
                    // Dumps the freight for 2 seconds, when done signals event and goes to next state
                    robot.intake.set(RobotParams.INTAKE_POWER_DUMP, 1.25, event);
                    sm.waitForSingleEvent(event, State.DRIVE_INTO_WAREHOUSE);
                    break;

                case DRIVE_INTO_WAREHOUSE:
                    //fire and forget with lowering arm
                    robot.arm.setLevel(1);
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                                event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(0.9, -2.6,  90.0),
                                robot.robotDrive.pathPoint(1.5, -2.6, 90.0)
                                );
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                                event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(0.9, 2.6,  90.0),
                                robot.robotDrive.pathPoint(1.5, 2.6, 90.0));
                    }
                    sm.waitForSingleEvent(event,State.PICK_UP_FREIGHT_FROM_WAREHOUSE );
                    break;

                case PICK_UP_FREIGHT_FROM_WAREHOUSE:
                    //if there are only 10 seconds left in autonomous we go to done because we are already in the warehouse
                    if(elapsedTime>=20){
                        sm.setState(State.DONE);
                    }
                    else if(expireTime==null){
                        expireTime = TrcUtil.getCurrentTime()+5;
                    }

                    if(grabberHasPickedBlock||TrcUtil.getCurrentTime()>expireTime){
                        //reset expireTIme so that next time we try to pick up freight from warehouse it still works
                        expireTime= null;
                        robot.intake.set(0);
                        sm.setState(State.DRIVE_OUT_OF_WAREHOUSE);
                        //code to turn off holonomic drive
                        robot.robotDrive.driveBase.holonomicDrive(0.0, 0.0, 0.0);
                    }

                    else{
                        //robot is facing 90 so y is forward(i think)
                        //keep driving forward while doing the intake power pickup

                        robot.intake.set(RobotParams.INTAKE_POWER_PICKUP);
                        robot.robotDrive.driveBase.holonomicDrive(0.0, 1.0, 0.0);
                    }
                    break;
                case DRIVE_OUT_OF_WAREHOUSE:
                    if(elapsedTime>=20){
                        sm.setState(State.DONE);
                    }
                    else{
                        //back out to around the place where we start but still facing the warehouse
                        if(autoChoices.alliance==FtcAuto.Alliance.RED_ALLIANCE){
                            robot.robotDrive.purePursuitDrive.start(
                                    event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(0.5, -2.6,  90.0));
                        }
                        else{
                            robot.robotDrive.purePursuitDrive.start(
                                    event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(0.5, 2.6,  90.0));
                        }
                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);

                    }
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