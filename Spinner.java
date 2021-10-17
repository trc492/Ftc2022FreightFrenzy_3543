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
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidMotor;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcFtcLib.ftclib.FtcDcMotor;

public class Spinner
{
//    private FtcDcMotor motor;
//    private TrcPidController pidCtrl;
//    private TrcPidMotor pidMotor;
//
//    public Spinner()
//    {
//        motor = new FtcDcMotor("spinnerMotor");
//        pidCtrl = new TrcPidController(
//                "spinnerPidCtrl",
//                new TrcPidController.PidCoefficients(RobotInfo.SPINNER_KP, RobotInfo.SPINNER_KI, RobotInfo.SPINNER_KD),
//                RobotInfo.SPINNER_TOLERANCE, motor::getPosition);
//        pidMotor = new TrcPidMotor("spinnerPidMotor", motor, pidCtrl, )
//    }
    private enum State
    {
        START_SPINNING,
        MONITOR_ANGLE,
        DONE
    }

    private Robot robot;
    private FtcDcMotor motor;
    private TrcStateMachine<State> sm;
    private TrcTaskMgr.TaskObject spinTask;
    private double spinAngle;
    private double spinPower;
    private TrcEvent event;

    public Spinner(Robot robot)
    {
        this.robot = robot;
        motor = new FtcDcMotor("spinnerMotor");
        sm = new TrcStateMachine<>("spinner");
        spinTask = TrcTaskMgr.getInstance().createTask("SpinTask", this::spinTask);
    }

    public void spinWithPower(double power)
    {
        motor.set(power);
    }

    public void cancel()
    {
        stop();
        sm.stop();
        spinTask.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
    }

    public void stop()
    {
        spinWithPower(0.0);
    }

    public void spinWithAngle(double angle, double power)
    {
        spinAngle = motor.getPosition()*RobotInfo.SPINNER_DEGREES_PER_COUNT + angle;
        spinPower = power;
        spinTask.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        sm.start(State.START_SPINNING);
    }

    private void spinTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else {
            boolean traceState = true;

            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state) {
                case START_SPINNING:
                    motor.set(spinPower);
                    sm.setState(State.MONITOR_ANGLE);
                    break;

                case MONITOR_ANGLE:
                    if (motor.getPosition() * RobotInfo.SPINNER_DEGREES_PER_COUNT >= spinAngle) {
                        stop();
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                    cancel();
                    break;
            }
        }
    }
}   //Spinner
