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
import TrcCommonLib.trclib.TrcHolonomicPurePursuitDrive;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcFtcLib.ftclib.FtcTensorFlow;

class CmdAutoTest implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoNearCarousel";

    private enum State
    {
        START,
        DO_PURE_PURSUIT,

//        FIND_OUR_GAME_PIECE,
//        DRIVE_TO_OUR_GAME_PIECE,
//        DO_INTAKE,

        DONE
    }   //enum State

    private final Robot robot;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final TrcHolonomicPurePursuitDrive purePursuitDrive;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    CmdAutoTest(Robot robot)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s", robot);

        this.robot = robot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        purePursuitDrive = new TrcHolonomicPurePursuitDrive(
            "purePursuitDrive", robot.robotDrive.driveBase,
            RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE, RobotParams.PPD_TURN_TOLERANCE,
            robot.robotDrive.yPosPidCoeff, robot.robotDrive.turnPidCoeff, robot.robotDrive.velPidCoeff);
        purePursuitDrive.setMsgTracer(robot.globalTracer);
        robot.arm.setLevel(1);
        sm.start(State.START);
    }   //CmdAutoTest

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
        Double expireTime = null;
        FtcTensorFlow.TargetInfo targetInfo = null;
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
                //put in code to test robot driving up to game piece  if it sees it
                case START:
                    robot.robotDrive.driveBase.setFieldPosition(RobotParams.STARTPOS_RED_NEAR );
                    sm.setState(State.DO_PURE_PURSUIT);
                    break;

                case DO_PURE_PURSUIT:
                    robot.robotDrive.purePursuitDrive.start(
                        event, 20.0, robot.robotDrive.driveBase.getFieldPosition(), false,
//                        RobotParams.ROBOT_MAX_VELOCITY/10.0, RobotParams.ROBOT_MAX_ACCELERATION,
                        robot.robotDrive.pathPoint(-2.5, 0.0, 90.0),
                        robot.robotDrive.pathPoint(0.5, 0.0, 90.0),
                        robot.robotDrive.pathPoint(0.5, -1.6, 90.0));
                    robot.robotDrive.purePursuitDrive.setWaypointEventHandler(
                        (i, p) -> {robot.globalTracer.traceInfo("*** TEST RED ***", "index=%d,waypoint=%s", i, p);});
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

//                case FIND_OUR_GAME_PIECE:
//
//                    targetInfo = robot.vision.getBestDetectedTargetInfo(Vision.LABEL_DUCK);
//                    if(targetInfo!=null){
//                        sm.setState(CmdAutoTest.State.DRIVE_TO_OUR_GAME_PIECE);
//                    }
//                    else  if(expireTime==null){
//                        expireTime = TrcUtil.getCurrentTime()+3;
//                    }
//                    else if (TrcUtil.getCurrentTime()>expireTime){
//                        //if time is up and we still havent found the thing than give up and do parking
//                        sm.setState(State.DONE);
//                    }
//
//
//                    break;
//
//                case DRIVE_TO_OUR_GAME_PIECE:
//                    // Call pure pursuit using robot field position +
//                    //after tuning distanceFromCenter will be real world distance from robot where robot is (0, 0)
//                    //turns such that the robot is inline with the game piece and moves forward
//                    //assumes distance from game object gives distance from computer to actual object
//
//
//                    //keep angle because it will go to a point and then turn
//                    //use incremental path so then it doesnt matter where robot is and we can just use distanceFromCamera
//
//                    TrcPose2D ourGamePiecePosition =
//                            new TrcPose2D(
//                                    //minus bc robot is facing backwards in its y orientationso if it says target at  right target actually at left
//                                    targetInfo.distanceFromCamera.x,
//                                    //robot is in opposite y orientation as the field
//                                    targetInfo.distanceFromCamera.y,
//                                    targetInfo.angle);
//                    robot.robotDrive.purePursuitDrive.start(
//                            event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), true,
//                            ourGamePiecePosition);
//
//                    sm.waitForSingleEvent(event, State.DONE);
//                    break;
//
//                case DO_INTAKE:
//                    robot.intake.set(RobotParams.INTAKE_POWER_PICKUP, 1.25, event);
//                    sm.waitForSingleEvent(event, State.DONE);
//
//                    break;

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

}   //class CmdAutoTest
