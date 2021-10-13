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

package team3543;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import TrcCommonLib.trclib.TrcHashMap;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcVuforia;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This class implements Vuforia Vision for the game season. It creates and initializes all the vision target info
 * as well as providing info for the robot, camera and the field. It also provides methods to get the location of the
 * robot and detected targets.
 */
public class VuforiaVision
{
    private static final int IMAGE_WIDTH = 1280;    //in pixels
    private static final int IMAGE_HEIGHT = 720;    //in pixels
    private static final int FRAME_QUEUE_CAPACITY = 2;

    private static final String blueStorageName = "Blue Storage";
    private static final String blueAllianceWallName = "Blue Alliance Wall";
    private static final String redStorageName = "Red Storage";
    private static final String redAllianceWallName = "Red Alliance Wall";
    private static final TrcHashMap<String, TrcRevBlinkin.LEDPattern> targetLEDPatternMap =
        new TrcHashMap<String, TrcRevBlinkin.LEDPattern>()
            .add(blueStorageName, TrcRevBlinkin.LEDPattern.SolidBlue)
            .add(blueAllianceWallName, TrcRevBlinkin.LEDPattern.FixedStrobeBlue)
            .add(redStorageName, TrcRevBlinkin.LEDPattern.SolidRed)
            .add(redAllianceWallName, TrcRevBlinkin.LEDPattern.FixedStrobeRed);
    //
    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here.
    //
    // Height of the center of the target image above the floor.
    private static final float mmTargetHeight = 6.0f * (float)TrcUtil.MM_PER_INCH;
    private static final float halfField = (float)(70.5 * TrcUtil.MM_PER_INCH);
    private static final float fullTile = (float)(23.75 * TrcUtil.MM_PER_INCH);
    private static final float halfTile = (float)(fullTile/2.0);
    private static final float oneAndHalfTile = (float)(fullTile*1.5);

    private final Robot robot;
    private final FtcVuforia vuforia;
    private final VuforiaTrackable[] imageTargets;
    private String lastImageName;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     * @param vuforia specifies the FtcVuforia object.
     * @param cameraLocation specifies the camera location.
     */
    public VuforiaVision(Robot robot, FtcVuforia vuforia, OpenGLMatrix cameraLocation)
    {
        this.robot = robot;
        this.vuforia = vuforia;
        vuforia.configVideoSource(IMAGE_WIDTH, IMAGE_HEIGHT, FRAME_QUEUE_CAPACITY);
        /*
         * In order for localization to work, we need to tell the system where each target is on the field, and where
         * the camera resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */
        OpenGLMatrix blueStorageLocation = OpenGLMatrix
            .translation(-halfField, oneAndHalfTile, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        OpenGLMatrix blueAllianceWallLocation = OpenGLMatrix
            .translation(halfTile, halfField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        OpenGLMatrix redStorageLocation = OpenGLMatrix
            .translation(-halfField, -oneAndHalfTile, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        OpenGLMatrix redAllianceWallLocation = OpenGLMatrix
            .translation(halfTile, -halfField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        //
        // Create and initialize all image targets.
        //
        FtcVuforia.TargetInfo[] imageTargetsInfo =
        {
            new FtcVuforia.TargetInfo(0, blueStorageName, false, blueStorageLocation),
            new FtcVuforia.TargetInfo(1, blueAllianceWallName, false, blueAllianceWallLocation),
            new FtcVuforia.TargetInfo(2, redStorageName, false, redStorageLocation),
            new FtcVuforia.TargetInfo(3, redAllianceWallName, false, redAllianceWallLocation)
        };

        vuforia.addTargetList(RobotInfo.TRACKABLE_IMAGES_FILE, imageTargetsInfo, cameraLocation);
        imageTargets = new VuforiaTrackable[imageTargetsInfo.length];
        for (int i = 0; i < imageTargets.length; i++)
        {
            imageTargets[i] = vuforia.getTarget(imageTargetsInfo[i].name);
        }

        if (robot.blinkin != null)
        {
            robot.blinkin.setNamedPatternMap(targetLEDPatternMap);
        }
    }   //VuforiaVision

    /**
     * This method enables/disables Vuforia Vision.
     *
     * @param enabled specifies true to enable Vuforia Vision, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        vuforia.setTrackingEnabled(enabled);
    }   //setEnabled

    /**
     * This method returns the name of the image last seen.
     *
     * @return last seen image name.
     */
    public String getLastSeenImageName()
    {
        return lastImageName;
    }   //getLastSeenImageName

    /**
     * This method returns the vector of the given target location object.
     *
     * @param location specifies the target location.
     * @return target location vector.
     */
    public VectorF getLocationTranslation(OpenGLMatrix location)
    {
        return location.getTranslation();
    }   //getLocationTranslation

    /**
     * This method returns the orientation of the given target location object.
     *
     * @param location specifies the target location.
     * @return target orientation.
     */
    public Orientation getLocationOrientation(OpenGLMatrix location)
    {
        return Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
    }   //getLocationOrientation

    /**
     * This method returns the robot location computed with the detected target.
     *
     * @param targetName specifies the detected target name.
     * @return robot location.
     */
    public OpenGLMatrix getRobotLocation(String targetName)
    {
        OpenGLMatrix robotLocation = null;
        VuforiaTrackable target = vuforia.getTarget(targetName);

        if (target != null)
        {
            robotLocation = vuforia.getRobotLocation(target);

            if (robot.blinkin != null && !Robot.Preferences.useBlinkinFlashLight)
            {
                if (robotLocation != null)
                {
                    robot.blinkin.setPatternState(targetName, true);
                }
                else
                {
                    robot.blinkin.reset();
                }
            }
        }

        return robotLocation;
    }   //getRobotLocation

    /**
     * This method returns the robot location computed with the detected target.
     *
     * @param targetName specifies the detected target name.
     * @param exclude specifies true to exclude the specified target.
     * @return robot location.
     */
    public OpenGLMatrix getRobotLocation(String targetName, boolean exclude)
    {
        OpenGLMatrix robotLocation = null;

        if (targetName == null || exclude)
        {
            for (VuforiaTrackable target: imageTargets)
            {
                String name = target.getName();
                boolean isMatched = targetName == null || !targetName.equals(name);

                if (isMatched && vuforia.isTargetVisible(target))
                {
                    // getRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix location = vuforia.getRobotLocation(target);
                    if (location != null)
                    {
                        robotLocation = location;
                        lastImageName = name;
                    }
                    break;
                }
            }
        }
        else
        {
            robotLocation = getRobotLocation(targetName);
            if (robotLocation != null)
            {
                lastImageName = targetName;
            }
        }

        if (robot.blinkin != null && !Robot.Preferences.useBlinkinFlashLight)
        {
            if (robotLocation != null)
            {
                robot.blinkin.setPatternState(lastImageName, true);
            }
            else
            {
                robot.blinkin.reset();
            }
        }

        return robotLocation;
    }   //getRobotLocation

    /**
     * This method returns the robot field position.
     *
     * @param targetName specifies the detected target name.
     * @param exclude specifies true to exclude the specified target.
     * @return robot field position.
     */
    public TrcPose2D getRobotPose(String targetName, boolean exclude)
    {
        OpenGLMatrix robotLocation = getRobotLocation(targetName, exclude);
        VectorF translation = robotLocation == null? null: getLocationTranslation(robotLocation);
        Orientation orientation = robotLocation == null? null: getLocationOrientation(robotLocation);
        //
        // The returned RobotPose have the X axis pointing from the audience side to the back of the field,
        // the Y axis pointing from the red alliance to the blue alliance and the direction of the Y axis
        // is zero degree and increases in the clockwise direction.
        // Vuforia's orientation is such that facing the negative side of the X axis is 0 degree, clockwise gives
        // you negative angle and anti-clockwise gives you positive angle.
        // Robot's orientation is such that facing the positive side of the Y axis is 0 degree, clockwise gives you
        // positive angle and anti-clockwise gives you negative angle.
        // In order to translate Vuforia's orientation to the robot's orientation, we first negate the vuforia angle
        // to make rotation agree with the robot's rotation. Then we subtract 90 degrees from the angle.
        //
        return (translation == null || orientation == null)? null:
            new TrcPose2D(translation.get(0)/TrcUtil.MM_PER_INCH, translation.get(1)/TrcUtil.MM_PER_INCH,
                          -orientation.thirdAngle - 90.0);
    }   //getRobotPose

}   //class VuforiaVision
