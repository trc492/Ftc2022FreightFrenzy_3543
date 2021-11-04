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

import TrcFtcLib.ftclib.FtcServo;

class OdometryWheelDeployer
{
    private final FtcServo yLeftDeployer;
    private final FtcServo yRightDeployer;
    private final FtcServo xDeployer;
    private boolean deployed = false;

    public OdometryWheelDeployer()
    {
        yLeftDeployer = new FtcServo(RobotParams.HWNAME_YLEFT_ODW_DEPLOYOR);
        yRightDeployer = new FtcServo(RobotParams.HWNAME_YRIGHT_ODW_DEPLOYOR);
        xDeployer = new FtcServo(RobotParams.HWNAME_X_ODW_DEPLOYOR);
    }   //OdometryWheelDeployer

    public void retract()
    {
        yLeftDeployer.setPosition(RobotParams.ODWHEEL_YLEFT_RETRACT_POS);
        yRightDeployer.setPosition(RobotParams.ODWHEEL_YRIGHT_RETRACT_POS);
        xDeployer.setPosition(RobotParams.ODWHEEL_X_RETRACT_POS);
        deployed = false;
    }   //retract

    public void deploy()
    {
        yLeftDeployer.setPosition(RobotParams.ODWHEEL_YLEFT_EXTEND_POS);
        yRightDeployer.setPosition(RobotParams.ODWHEEL_YRIGHT_EXTEND_POS);
        xDeployer.setPosition(RobotParams.ODWHEEL_X_EXTEND_POS);
        deployed = true;
    }   //deploy

    public boolean isDeployed()
    {
        return deployed;
    }   //isDeployed

}   //class OdometryWheelDeployer
