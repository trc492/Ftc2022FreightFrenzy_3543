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

class CmdAutoNearCarouselWithDuck implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoNearCarouselWithDuck";
    private static final double PARK_WAREHOUSE_TIME = 8.0;

    private enum State
    {
        START_DELAY,
        DRIVE_TO_ALLIANCE_SHIPPING_HUB,
        DUMP_FREIGHT,

        DRIVE_TO_CAROUSEL,
        GET_TO_CAROUSEL,
        SPIN_CAROUSEL,

        POSITION_TO_FIND_THE_DUCK,
        ALIGN_WITH_WALL,
        FIND_THE_DUCK,
        STAY,
        GO_TO_PICKUP_POSITION,
        GO_PICKUP_DUCK,
        DONE_PICKUP_DUCK,
//        TURN_AROUND,

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
    private TrcPose2D duckPose = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAutoNearCarouselWithDuck(Robot robot, FtcAuto.AutoChoices autoChoices)
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

                    robot.arm.setLevel(duckPosition);
                    //
                    // Do start delay if any.
                    //
                    if (autoChoices.startDelay < 1.0)
                    {
                        // Give the arm at least one second of head start so that it will reach correct level when
                        // the robot is at the alliance shipping hub.
                        autoChoices.startDelay = 1.0;
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
                    // Drive to alliance shipping hub. We are coming from starting position or somewhere near carousel.
                    // Note: the smaller the number the closer to the hub.
                    double hubHeading, distanceToHub;
                    double hubX, hubY;
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        hubHeading = 30.0;
                        distanceToHub = duckPosition == 3? 0.7: duckPosition == 2? 0.9: 0.85;
                        hubX = -0.5;
                        hubY = -1.0;
                    }
                    else
                    {
                        hubHeading = 180.0 - 30.0;
                        distanceToHub = duckPosition == 3? 0.7: duckPosition == 2? 0.9: 0.85;
                        hubX = -0.5;
                        hubY = 1.0;
                    }
                    hubX -= distanceToHub*Math.sin(Math.toRadians(hubHeading));
                    hubY -= distanceToHub*Math.cos(Math.toRadians(hubHeading));

                    if (deliveringDuck)
                    {
                        // We are coming from the duck pickup point somewhere near the carousel.
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-1.5, -2.5, hubHeading),
                                robot.robotDrive.pathPoint(hubX , hubY, hubHeading));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-1.5, 2.5, hubHeading),
                                robot.robotDrive.pathPoint(hubX, hubY, hubHeading));
                        }
                    }
                    // We are coming from the starting position.
                    else if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        // Add one extra point to make sure it doesn't overshoot the turn.
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-1.4, -2.2, hubHeading),
                            robot.robotDrive.pathPoint(hubX, hubY, hubHeading));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-1.4, 2.2, hubHeading),
                            robot.robotDrive.pathPoint(hubX, hubY, hubHeading));
                    }
                    // Raise arm to the detected duck level at the same time.
                    robot.arm.setLevel(duckPosition);
                    // After we dump the freight to the right level for the bonus, any subsequent dumps will be to
                    // the top.
                    duckPosition = 3;
                    sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                    break;

                case DUMP_FREIGHT:
                    // Dumps the freight, when done, signals event and goes to next state.
                    robot.intake.setPower(RobotParams.INTAKE_POWER_DUMP, RobotParams.INTAKE_DUMP_TIME, event);
                    sm.waitForSingleEvent(
                        event, deliveringDuck? State.DRIVE_TO_ALLIANCE_STORAGE_UNIT: State.DRIVE_TO_CAROUSEL);
                    break;

                case DRIVE_TO_CAROUSEL:
                    robot.arm.setLevel(0.5, 1);
                    // Drive to the carousel from the alliance shipping hub.
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
                    // Spin the carousel a bit extra time so duck doesn't go far.
                    robot.spinner.set(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            RobotParams.SPINNER_POWER_RED: RobotParams.SPINNER_POWER_BLUE,
                        RobotParams.SPINNER_TIME + 1.0, event);
                    sm.waitForSingleEvent(event, State.POSITION_TO_FIND_THE_DUCK);
                    break;

                case POSITION_TO_FIND_THE_DUCK:
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-1.5, -1.5, -90.0),
                            robot.robotDrive.pathPoint(-1.0, -2.5, -90.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-1.5, 1.5, -90.0),
                            robot.robotDrive.pathPoint(-1.0, 2.5, -90.0));
                    }
                    retryCount = 0;
                    sm.waitForSingleEvent(event, State.ALIGN_WITH_WALL);
                    break;

                case ALIGN_WITH_WALL:
                    robot.robotDrive.driveBase.holonomicDrive(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE? -0.3: 0.3, 0.0, 0.0);
                    timer.set(0.5, event);
                    sm.waitForSingleEvent(event, State.STAY);
                    break;

                case STAY:
                    // CodeReview: do we really need this?
                    robot.robotDrive.driveBase.stop();
                    timer.set(1.0, event);
                    sm.waitForSingleEvent(event, State.FIND_THE_DUCK);

                case FIND_THE_DUCK:
                    targetInfo = robot.vision.getClosestDuckInfo();
                    robot.arm.setLevel(0);
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
                        sm.setState(State.GO_TO_PICKUP_POSITION);
                    }
                    else if (expireTime == null)
                    {
                        expireTime = TrcUtil.getCurrentTime() + 3.0;
                    }
                    else if (TrcUtil.getCurrentTime() > expireTime)
                    {
                        if (retryCount == 0)
                        {
                            // Did not find the duck the first time, zoom in and look again.
                            robot.globalTracer.traceInfo(moduleName, "<<<<< Zoom in and look again!");
                            robot.speak("Zoom in and look again.");
                            robot.vision.setTensorFlowZoomFactor(1.5);
                            expireTime = null;
                            retryCount++;
                            // stay in this state to retry.
                        }
                        else if (retryCount == 1)
                        {
                            // Did not find the duck the second time, go forward and look again.
                            robot.globalTracer.traceInfo(moduleName, "<<<<< Drive forward and look again!");
                            robot.speak("Drive forward and look again.");
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), true,
                                new TrcPose2D(0.0, 10.0, 0.0));
                            expireTime = null;
                            retryCount++;
                            // stay in this state to retry.
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

                case GO_TO_PICKUP_POSITION:
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE &&
                        targetInfo.distanceFromCamera.x < -1.0 ||
                        autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE &&
                        targetInfo.distanceFromCamera.x > 1.0)
                    {
                        // Duck is too close to the wall, must go to position perpendicular to the wall to pick it up.
                        double distanceToDuck = Math.max(targetInfo.distanceFromCamera.y, RobotParams.FULL_TILE_INCHES);

                        duckPose = new TrcPose2D(
                            robot.robotDrive.driveBase.getXPosition() - targetInfo.distanceFromCamera.y,
                            robot.robotDrive.driveBase.getYPosition() + targetInfo.distanceFromCamera.x);
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(duckPose.x, duckPose.y + distanceToDuck, 180.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(duckPose.x, duckPose.y - distanceToDuck, 0.0));
                        }
                        sm.waitForSingleEvent(event, State.GO_PICKUP_DUCK);
                    }
                    else
                    {
                        sm.setState(State.GO_PICKUP_DUCK);
                    }
                    break;

                case GO_PICKUP_DUCK:
                    //robot.intake.autoAssist(RobotParams.INTAKE_POWER_PICKUP);
                    robot.intake.setPower(RobotParams.INTAKE_POWER_PICKUP);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    if (duckPose != null)
                    {
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(duckPose.x, duckPose.y + 6.0, 180.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(duckPose.x, duckPose.y - 6.0, 0.0));
                        }
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), true,
                            new TrcPose2D(targetInfo.distanceFromCamera.x, targetInfo.distanceFromCamera.y, 0.0));
                    }
                    sm.waitForSingleEvent(event, State.DONE_PICKUP_DUCK);
                    break;

                case DONE_PICKUP_DUCK:
                    // Keep spinning the intake at low power to keep the duck from falling out.
                    robot.intake.setPower(0.3);
                    robot.robotDrive.purePursuitDrive.cancel();
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                    deliveringDuck = true;
                    sm.setState(State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    break;

//                case TURN_AROUND:
//                    robot.robotDrive.purePursuitDrive.start(
//                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                        new TrcPose2D(
//                            robot.robotDrive.driveBase.getXPosition(), robot.robotDrive.driveBase.getYPosition(), 45.0));
//                    sm.waitForSingleEvent(event,State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
//                    break;

                case DRIVE_TO_ALLIANCE_STORAGE_UNIT:
                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        // We are not parking anywhere, just stop and be done.
                        sm.setState(State.DONE);
                    }
                    // If we don't have enough time to go to the warehouse, park at the storage unit instead.
                    else if (autoChoices.parking == FtcAuto.Parking.WAREHOUSE_PARKING &&
                             30.0 - elapsedTime > PARK_WAREHOUSE_TIME)
                    {
                        // We are parking at the warehouse.
                        sm.setState(State.DRIVE_TO_WAREHOUSE);
                    }
                    else
                    {
                        // Drive to storage unit from the alliance shipping hub.
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            if(deliveringDuck)
                            {
                                robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-1.5, -1.5, 90.0),
                                    robot.robotDrive.pathPoint(-2.45, -1.6, 90.0));
                            }
                            else
                            {
                                robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-1.5, 1.5, 90.0),
                                    robot.robotDrive.pathPoint(-2.45, 1.6, 90.0));
                            }
                        }
                        else
                        {
                            // CodeReview: why is this so different from RED_ALLIANCE?
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.5, 1.4, 90.0));
                        }
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case DRIVE_TO_WAREHOUSE:
                    // We are heading to the warehouse but we could be coming from picking up duck or alliance hub.
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-1.5, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, 0.0, 90.0),
                            robot.robotDrive.pathPoint(0.5, -1.6, 90.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(-1.5, 0.0, 90.0),
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
                    state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive, robot.robotDrive.purePursuitDrive,
                    null);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoNearCarouselWithDuck
