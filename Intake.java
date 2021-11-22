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
import TrcCommonLib.trclib.TrcTimer;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcDistanceSensor;

class Intake extends FtcDcMotor
{
    private static final String moduleName = "Intake";
    private static final double FREIGHT_THRESHOLD = 3.0;
    private static final double[] thresholds = {FREIGHT_THRESHOLD};
    private final FtcDistanceSensor sensor;
    private final TrcAnalogSensorTrigger<FtcDistanceSensor.DataType> sensorTrigger;
    private final TrcTimer timer;
    private TrcEvent onFinishEvent = null;
    private boolean gotFreight = false;

    public Intake(String instanceName)
    {
        super(instanceName);
        sensor = new FtcDistanceSensor(moduleName + "Sensor");
        sensorTrigger = new TrcAnalogSensorTrigger<>(
            moduleName + "SensorTrigger", sensor, 0, FtcDistanceSensor.DataType.DISTANCE_INCH, thresholds,
            this::triggerHandler, false);
        timer = new TrcTimer(moduleName + "Timer");
    }   //Intake

    public void pickUpFreight(double power, TrcEvent event, double timeout)
    {
        gotFreight =
            sensor.getRawData(0, FtcDistanceSensor.DataType.DISTANCE_INCH).value > FREIGHT_THRESHOLD;
        if (gotFreight)
        {
            event.signal();
        }
        else
        {
            super.set(power);
            this.onFinishEvent = event;
            timer.set(timeout, this::timeoutHandler);
            sensorTrigger.setEnabled(true);
        }
    }   //pickupFreight

    public boolean hasFreight()
    {
        return gotFreight;
    }   //hasFreight

    private void cancel()
    {
        super.set(0.0);
        onFinishEvent.signal();
        sensorTrigger.setEnabled(false);
    }   //cancel

    private void triggerHandler(int currZone, int prevZone, double zoneValue)
    {
        if (currZone < prevZone)
        {
            // We got freight.
            cancel();
            gotFreight = true;
        }
    }   //triggerHandler

    private void timeoutHandler(Object timer)
    {
        cancel();
    }   //timeoutHandler

}   //class Intake
