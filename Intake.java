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

import TrcCommonLib.trclib.TrcAnalogSensorTrigger;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcNotifier;
import TrcCommonLib.trclib.TrcTimer;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcDistanceSensor;

/**
 * This class implements an auto-assist intake subsystem. It contains a motor and a distance sensor that detects if
 * the intake has picked up freight. It provides the pickupFreight method that allows the caller to call the intake
 * subsystem to pickup freight on a press of a button and the intake subsystem will stop itself once freight is
 * detected in the intake. While it provides the auto-assist functionality to pickup freight, it also supports
 * exclusive subsystem access by implementing TrcExclusiveSubsystem. This enables the intake subsystem to be aware
 * of multiple callers access to the subsystem. While one caller starts the intake for an operation, nobody can
 * access it until the previous caller is done with the operation.
 */
class Intake extends FtcDcMotor implements TrcExclusiveSubsystem
{
    private static final String moduleName = "Intake";
    private static final double FREIGHT_THRESHOLD = 3.8;    //in cm
    private static final double[] thresholds = {FREIGHT_THRESHOLD};
    private final FtcDistanceSensor sensor;
    private final TrcAnalogSensorTrigger<FtcDistanceSensor.DataType> distanceTrigger;
    private final TrcTimer timer;
    private TrcEvent onFinishEvent = null;
    private TrcNotifier.Receiver onFinishCallback = null;
    private String ownerID = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName
     */
    public Intake(String instanceName)
    {
        super(instanceName);
        sensor = new FtcDistanceSensor(moduleName + "Sensor");
        distanceTrigger = new TrcAnalogSensorTrigger<>(
            moduleName + "SensorTrigger", sensor, 0, FtcDistanceSensor.DataType.DISTANCE_CM, thresholds,
            this::triggerHandler, false);
        timer = new TrcTimer(moduleName + "Timer");
    }   //Intake

    /**
     * This method spins the intake with the given power. It will only do this if nobody acquires exclusive access
     * to the intake already.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param power specifies the power value to spin the intake.
     */
    public void set(String owner, double power)
    {
        if (validateOwnership(owner))
        {
            super.set(power);
        }
    }   //set

    /**
     * This method spins the intake with the given power. It will only do this if nobody acquires exclusive access
     * to the intake already.
     *
     * @param power specifies the power value to spin the intake.
     */
    @Override
    public void set(double power)
    {
        set(null, power);
    }   //set

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.The value can be power or velocity percentage depending on whether the motor controller is in
     * power mode or velocity mode.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void set(String owner, double value, double time, TrcEvent event)
    {
        if (validateOwnership(owner))
        {
            super.set(value, time, event);
        }
    }   //set

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once freight is detected in the intake at which time the given event will be signaled
     * or it will notify the caller's handler.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param power specifies the power value to spin the intake.
     * @param event specifies the event to signal when there is freight detected in the intake.
     * @param callback specifies the callback handler to call when there is freight detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasFreight to figure out if it has given up.
     */
    public void pickupFreight(String owner, double power, TrcEvent event, TrcNotifier.Receiver callback, double timeout)
    {
        //
        // This is an auto-assist pickup, make sure the caller has ownership.
        //
        if (validateOwnership(owner))
        {
            ownerID = owner;

            if (hasFreight())
            {
                if (event != null)
                {
                    event.signal();
                }

                if (callback != null)
                {
                    callback.notify(ownerID);
                }

                ownerID = null;
            }
            else
            {
                super.set(power);
                this.onFinishEvent = event;
                this.onFinishCallback = callback;
                if (timeout > 0.0)
                {
                    timer.set(timeout, this::timeoutHandler);
                }
                distanceTrigger.setEnabled(true);
            }
        }
    }   //pickupFreight

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once freight is detected in the intake.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param power specifies the power value to spin the intake.
     */
    public void pickupFreight(String owner, double power)
    {
        pickupFreight(owner, power, null, null, 0.0);
    }   //pickupFreight

    /**
     * This method reads the distance value from the distance sensor.
     *
     * @return distance value read from the distance sensor.
     */
    public double getDistance()
    {
        return sensor.getRawData(0, FtcDistanceSensor.DataType.DISTANCE_CM).value;
    }   //getDistance

    /**
     * This method checks if there is freight in the intake.
     *
     * @return true if there is freight in the intake, false otherwise.
     */
    public boolean hasFreight()
    {
        return getDistance() < FREIGHT_THRESHOLD;
    }   //hasFreight

    /**
     * This method is called internally either at the end of the timeout or when freight is detected to stop the
     * intake's spinning and signal or notify the caller for completion.
     */
    private void cancel()
    {
        super.set(0.0);

        if (onFinishEvent != null)
        {
            onFinishEvent.signal();
        }

        if (onFinishCallback != null)
        {
            onFinishCallback.notify(ownerID);
        }

        ownerID = null;
        distanceTrigger.setEnabled(false);
    }   //cancel

    /**
     * This method is called when freight is detected in the intake.
     *
     * @param currZone specifies the current threshold zone.
     * @param prevZone specifies the previous threshold zone.
     * @param zoneValue specifies the sensor value.
     */
    private void triggerHandler(int currZone, int prevZone, double zoneValue)
    {
        if (currZone < prevZone)
        {
            // We got freight.
            cancel();
        }
    }   //triggerHandler

    /**
     * This method is called when timeout expires.
     *
     * @param timer specifies the timer that has expired.
     */
    private void timeoutHandler(Object timer)
    {
        cancel();
    }   //timeoutHandler

}   //class Intake
