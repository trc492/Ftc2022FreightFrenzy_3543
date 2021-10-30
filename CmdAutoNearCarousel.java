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
    private enum State
    {
        //assume duckie is preloaded
        START_DELAY,
        DRIVE_TO_CAROUSEL,
        SPIN_CAROUSEL,
        DRIVE_TO_SHIPPING_HUB,
        DUMP_FREIGHT,
        DRIVE_STORAGE_UNIT,

        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoNearCarousel";

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
        robot.setFlashLightOn(false);
        if (robot.pidDrive.isActive())
        {
            robot.pidDrive.cancel();
        }
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

            switch (state)
            {
                case START_DELAY:
                    //
                    // Do start delay if any.
                    //call vision
                    if(robot.vision!=null&&robot.vision.isTensorFlowInitialized()){
                        duckPosition = robot.vision.getLastDuckPosition();
                        robot.globalTracer.traceInfo(moduleName, "Duck found at position %d", duckPosition);
                        if (duckPosition == 0) duckPosition = 2;
                    }
                    //
                    if (autoChoices.startDelay == 0.0)
                    {
                        sm.setState(State.DRIVE_TO_CAROUSEL);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_CAROUSEL);
                        break;
                    }
                case DRIVE_TO_CAROUSEL:
                    yTarget= RobotInfo.CAROUSEL_LOCATION.y- robot.pidDrive.getAbsoluteTargetPose().y;
                    xTarget = RobotInfo.CAROUSEL_LOCATION.x - robot.pidDrive.getAbsoluteTargetPose().x;
                    State nextState=null;
                    robot.pidDrive.setRelativeTarget(xTarget, yTarget, 0, event);
                    sm.waitForSingleEvent(event,State.SPIN_CAROUSEL);
                    break;
                case SPIN_CAROUSEL:
                    robot.spinner.set(
                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                                    RobotInfo.SPINNER_POWER_RED: RobotInfo.SPINNER_POWER_BLUE,
                            RobotInfo.SPINNER_TIME, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SHIPPING_HUB);
                    break;

                case DRIVE_TO_SHIPPING_HUB:
                    //rotate arm to the right level while driving, i think it is equal to duckLevel-currentLevel
                    robot.arm.setPosition(RobotInfo.ARM_PRESET_LEVELS[duckPosition]);
                    //move to the shipping hub location
                    xTarget = RobotInfo.SHIPPING_HUB_LOCATION.x - robot.pidDrive.getAbsoluteTargetPose().x;
                    yTarget = RobotInfo.SHIPPING_HUB_LOCATION.y - robot.pidDrive.getAbsoluteTargetPose().y;
                    robot.pidDrive.setRelativeTarget(xTarget, yTarget, 0, event);
                    sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                    break;

                case DUMP_FREIGHT:
                    //dumps the block for 2 seconds, when done signals event and goes to next state
                    robot.intake.set(RobotInfo.INTAKE_POWER_DUMP, 2, event);
                    sm.waitForSingleEvent(event, State.DRIVE_STORAGE_UNIT);
                    break;
                case DRIVE_STORAGE_UNIT:
                    //find distance to drive from current position to target position o
                    xTarget = RobotInfo.STORAGE_UNIT_LOCATION.x - robot.pidDrive.getAbsoluteTargetPose().x;
                    yTarget = RobotInfo.STORAGE_UNIT_LOCATION.y - robot.pidDrive.getAbsoluteTargetPose().y;
                    robot.pidDrive.setRelativeTarget(xTarget, yTarget, 0, event);
                    sm.waitForSingleEvent(event, State.DONE);

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
