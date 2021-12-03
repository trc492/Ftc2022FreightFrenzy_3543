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

import TrcCommonLib.trclib.TrcIntake;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcDistanceSensor;

class Intake
{
    private final TrcIntake<?> intake;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param params specifies the parameters for the Intake subsystem.
     */
    public Intake(String instanceName, TrcIntake.Parameters params)
    {
        FtcDcMotor motor = new FtcDcMotor(instanceName + ".motor");
        FtcDistanceSensor sensor = RobotParams.Preferences.hasIntakeSensor?
            new FtcDistanceSensor(instanceName + ".sensor"): null;
        intake = new TrcIntake<>(
            instanceName, motor, params, sensor, 0, FtcDistanceSensor.DataType.DISTANCE_CM);
    }   //Intake

    /**
     * This method returns the Intake subsystem created.
     *
     * @return reference to the created Intake subsystem.
     */
    public TrcIntake<?> getIntakeInstance()
    {
        return intake;
    }   //getIntakeInstance

}   //class Intake
