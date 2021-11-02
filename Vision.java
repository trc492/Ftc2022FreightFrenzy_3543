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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHashMap;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcVuforia;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import java.util.Arrays;

/**
 * This class implements Vuforia/TensorFlow Vision for the game season. It creates and initializes all the vision
 * target info as well as providing info for the robot, camera and the field. It also provides methods to get the
 * location of the robot and detected targets.
 */
public class Vision
{
    public static final String blueStorageName = "Blue Storage";
    public static final String blueAllianceWallName = "Blue Alliance Wall";
    public static final String redStorageName = "Red Storage";
    public static final String redAllianceWallName = "Red Alliance Wall";
    public static final String LABEL_BALL = "Ball";
    public static final String LABEL_CUBE = "Cube";
    public static final String LABEL_DUCK = "Duck";
    public static final String LABEL_MARKER = "Marker";
    public static final String duckPos1 = "DuckPos1";
    public static final String duckPos2 = "DuckPos2";
    public static final String duckPos3 = "DuckPos3";

    private final Robot robot;
    private final TrcDbgTrace tracer;
    private final FtcVuforia vuforia;
    private static final TrcHashMap<String, TrcRevBlinkin.LEDPattern> targetLEDPatternMap =
        new TrcHashMap<String, TrcRevBlinkin.LEDPattern>()
            .add(duckPos1, TrcRevBlinkin.LEDPattern.SolidRed)
            .add(duckPos2, TrcRevBlinkin.LEDPattern.SolidGreen)
            .add(duckPos3, TrcRevBlinkin.LEDPattern.SolidBlue)
            .add(redStorageName, TrcRevBlinkin.LEDPattern.FixedStrobeRed)
            .add(blueStorageName, TrcRevBlinkin.LEDPattern.FixedStrobeBlue)
            .add(redAllianceWallName, TrcRevBlinkin.LEDPattern.FixedLightChaseRed)
            .add(blueAllianceWallName, TrcRevBlinkin.LEDPattern.FixedLightChaseBlue);
    private static final TrcRevBlinkin.LEDPattern[] ledPatternPriorities = {
        TrcRevBlinkin.LEDPattern.SolidRed,
        TrcRevBlinkin.LEDPattern.SolidGreen,
        TrcRevBlinkin.LEDPattern.SolidBlue,
        TrcRevBlinkin.LEDPattern.FixedStrobeRed,
        TrcRevBlinkin.LEDPattern.FixedStrobeBlue,
        TrcRevBlinkin.LEDPattern.FixedLightChaseRed,
        TrcRevBlinkin.LEDPattern.FixedLightChaseBlue};

    /**
     * Constructor: Create an instance of the object. Vision is required by both Vuforia and TensorFlow and must be
     * instantiated if either is used. However, to use either Vuforia or TensorFlow, one must explicitly initialize
     * them by calling the initVuforia or initTensorFlow methods respectively.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        this.robot = robot;
        this.tracer = TrcDbgTrace.getGlobalTracer();
        final String VUFORIA_LICENSE_KEY =
            "ARbBwjf/////AAABmZijKPKUWEY+uNSzCuTOUFgm7Gr5irDO55gtIOjsOXmhLzLEILJp45qdPrwMfoBV2Yh7F+Wh8iEjnSA" +
            "NnnRKiJNHy1T9Pr2uufETE40YJth10Twv0sTNSEqxDPhg2t4PJXwRImMaEsTE53fmcm08jT9qMso2+1h9eNk2b4x6DVKgBt" +
            "Tv5wocDs949Gkh6lRt5rAxATYYO9esmyKyfyzfFLMMpfq7/uvQQrSibNBqa13hJRmmHoM2v0Gfk8TCTTfP044/XsOm54u8k" +
            "dv0HfeMBC91uQ/NvWHVV5XCh8pZAzmL5sry1YwG8FSRNVlSAZ1zN/m6jAe98q6IxpwQxP0da/TpJoqDI7x4RGjOs1Areunf";
        FtcOpMode opMode = FtcOpMode.getInstance();
        int cameraViewId = !Robot.Preferences.showVuforiaView ? -1 :
            opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        //
        // If no camera view ID, do not activate camera monitor view to save power.
        //
        VuforiaLocalizer.Parameters vuforiaParams =
            cameraViewId == -1? new VuforiaLocalizer.Parameters(): new VuforiaLocalizer.Parameters(cameraViewId);

        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraName = opMode.hardwareMap.get(WebcamName.class, RobotInfo.HWNAME_WEBCAM);
        vuforiaParams.useExtendedTracking = false;
        vuforiaParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = new FtcVuforia(vuforiaParams);
    }   //Vision

    /**
     * This method sets up the Blinkin with a priority pattern list and a pattern name map.
     */
    public void setupBlinkin()
    {
        if (robot.blinkin != null)
        {
            robot.blinkin.setNamedPatternMap(targetLEDPatternMap);
            robot.blinkin.setPatternPriorities(ledPatternPriorities);
        }
    }   //setupBlinkin

    //
    // Vuforia Vision.
    //
    private static final int IMAGE_WIDTH = 640;     //in pixels
    private static final int IMAGE_HEIGHT = 480;    //in pixels
    private static final int FRAME_QUEUE_CAPACITY = 2;
    //
    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here.
    //
    // Height of the center of the target image above the floor.
    private static final float mmTargetHeight = 6.0f * (float)TrcUtil.MM_PER_INCH;
    private static final float halfField = (float)(RobotInfo.HALF_FIELD_INCHES * TrcUtil.MM_PER_INCH);
    private static final float fullTile = (float)(RobotInfo.FULL_TILE_INCHES * TrcUtil.MM_PER_INCH);
    private static final float halfTile = (float)(RobotInfo.HALF_TILE_INCHES * TrcUtil.MM_PER_INCH);
    private static final float oneAndHalfTile = (float)(fullTile*1.5);

    private boolean vuforiaInitialized = false;
    private VuforiaTrackable[] vuforiaImageTargets;
    private String lastVuforiaImageName;

    /**
     * This method must be called before any Vuforia related methods can be called or it may throw an exception.
     */
    public void initVuforia()
    {
        if (!vuforiaInitialized)
        {
            vuforia.configVideoSource(IMAGE_WIDTH, IMAGE_HEIGHT, FRAME_QUEUE_CAPACITY);
            /*
             * Create a transformation matrix describing where the camera is on the robot.
             *
             * Info:  The coordinate frame for the robot looks the same as the field.
             * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
             * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
             *
             * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
             * with the wide (horizontal) axis of the camera aligned with the X axis, and
             * the Narrow (vertical) axis of the camera aligned with the Y axis
             *
             * But, this example assumes that the camera is actually facing forward out the front of the robot.
             * So, the "default" camera position requires two rotations to get it oriented correctly.
             * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
             * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
             *
             * Finally the camera can be translated to its actual mounting position on the robot.
             *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
             */
            final float CAMERA_FORWARD_DISPLACEMENT =
                (float)((RobotInfo.ROBOT_LENGTH/2.0 - RobotInfo.CAMERA_FRONT_OFFSET)* TrcUtil.MM_PER_INCH);
            final float CAMERA_VERTICAL_DISPLACEMENT =
                (float)(RobotInfo.CAMERA_HEIGHT_OFFSET*TrcUtil.MM_PER_INCH);
            final float CAMERA_LEFT_DISPLACEMENT =
                (float)((RobotInfo.ROBOT_WIDTH/2.0 - RobotInfo.CAMERA_LEFT_OFFSET)*TrcUtil.MM_PER_INCH);
            OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 90, 90, 0));
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
            FtcVuforia.TargetInfo[] imageTargetsInfo = {
                new FtcVuforia.TargetInfo(0, blueStorageName, false, blueStorageLocation),
                new FtcVuforia.TargetInfo(1, blueAllianceWallName, false, blueAllianceWallLocation),
                new FtcVuforia.TargetInfo(2, redStorageName, false, redStorageLocation),
                new FtcVuforia.TargetInfo(3, redAllianceWallName, false, redAllianceWallLocation)
            };
            vuforia.addTargetList(RobotInfo.TRACKABLE_IMAGES_FILE, imageTargetsInfo, cameraLocationOnRobot);

            vuforiaImageTargets = new VuforiaTrackable[imageTargetsInfo.length];
            for (int i = 0; i < vuforiaImageTargets.length; i++)
            {
                vuforiaImageTargets[i] = vuforia.getTarget(imageTargetsInfo[i].name);
            }

            vuforiaInitialized = true;
        }
    }   //initVuforia

    /**
     * This method checks if Vuforia has been initialized.
     *
     * @return true if Vuforia is initialized, false otherwise.
     */
    public boolean isVuforiaInitialized()
    {
        return vuforiaInitialized;
    }   //isVuforiaInitialized

    /**
     * This method enables/disables Vuforia Vision.
     *
     * @param enabled specifies true to enable Vuforia Vision, false to disable.
     */
    public void setVuforiaEnabled(boolean enabled)
    {
        if (!vuforiaInitialized) throw new RuntimeException("Vuforia Vision is not initialized!");
        vuforia.setTrackingEnabled(enabled);
    }   //setVuforiaEnabled

    /**
     * This method returns the name of the image last seen.
     *
     * @return last seen image name.
     */
    public String getLastSeenVuforiaImageName()
    {
        return lastVuforiaImageName;
    }   //getLastSeenVuforiaImageName

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
        if (!vuforiaInitialized) throw new RuntimeException("Vuforia Vision is not initialized!");

        OpenGLMatrix robotLocation = null;
        VuforiaTrackable target = vuforia.getTarget(targetName);

        if (target != null)
        {
            robotLocation = vuforia.getRobotLocation(target);

            if (robot.blinkin != null)
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
        if (!vuforiaInitialized) throw new RuntimeException("Vuforia Vision is not initialized!");

        OpenGLMatrix robotLocation = null;

        if (targetName == null || exclude)
        {
            for (VuforiaTrackable target: vuforiaImageTargets)
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
                        lastVuforiaImageName = name;
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
                lastVuforiaImageName = targetName;
            }
        }

        if (robot.blinkin != null)
        {
            if (robotLocation != null)
            {
                robot.blinkin.setPatternState(lastVuforiaImageName, true);
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

    //
    // TensorFlow Vision.
    //
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] OBJECT_LABELS = {LABEL_BALL, LABEL_CUBE, LABEL_DUCK, LABEL_MARKER};
    private static final float TFOD_MIN_CONFIDENCE = 0.5f;
    private static final double ASPECT_RATIO_TOLERANCE_LOWER = 0.65;
    private static final double ASPECT_RATIO_TOLERANCE_UPPER = 1.2;
    // Target size is area of target rect.
    private static final double TARGET_SIZE_TOLERANCE_LOWER = 4000.0;
    private static final double TARGET_SIZE_TOLERANCE_UPPER = 22000.0;

    //tolerance for distance from center of screen to the target y


    private FtcTensorFlow tensorFlow = null;
    private int[] lastDuckPositions = null;

    /**
     * This method must be called before any TensorFlow related methods can be called or it may throw an exception.
     */
    public void initTensorFlow()
    {
        if (tensorFlow == null)
        {
            FtcOpMode opMode = FtcOpMode.getInstance();
            int tfodMonitorViewId = !Robot.Preferences.showTensorFlowView ? -1 :
                opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            //
            // If no TFOD monitor view ID, do not activate camera monitor view to save power.
            //
            TFObjectDetector.Parameters tfodParams =
                tfodMonitorViewId == -1?
                    new TFObjectDetector.Parameters() : new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParams.minResultConfidence = TFOD_MIN_CONFIDENCE;
            tfodParams.isModelTensorFlow2 = true;
            tfodParams.inputSize = 320;
        }
    }   //initTensorFlow

    /**
     * This method checks if TensorFlow has been initialized.
     *
     * @return true if Vuforia is initialized, false otherwise.
     */
    public boolean isTensorFlowInitialized()
    {
        return tensorFlow != null;
    }   //isTensorFlowInitialized

    /**
     * This method enables/disables TensorFlow.
     *
     * @param enabled specifies true to enable TensorFlow, false to disable.
     */
    public void setTensorFlowEnabled(boolean enabled)
    {
        if (tensorFlow == null) throw new RuntimeException("TensorFlow Vision is not initialized!");

        tensorFlow.setEnabled(enabled);
        if (enabled)
        {
            tensorFlow.setZoom(1.0, 16.0/9.0);
        }
    }   //setTensorFlowEnabled

    /**
     * This method shuts down TensorFlow.
     */
    public void tensorFlowShutdown()
    {
        setTensorFlowEnabled(false);
        tensorFlow.shutdown();
        tensorFlow = null;
    }   //tensorFlowShutdown

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing confidence.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has higher confidence than b, 0 if a and b have equal confidence, positive value
     *         if a has lower confidence than b.
     */
    private int compareConfidence(FtcTensorFlow.TargetInfo a, FtcTensorFlow.TargetInfo b)
    {
        return (int)((b.confidence - a.confidence)*100);
    }   //compareConfidence

    /**
     * This method returns an array of detected targets from TensorFlow vision.
     *
     * @param label specifies the label of the targets to detect for, can be null for detecting any target.
     * @param filter specifies the filter to call to filter out false positive targets.
     * @return array of detected target info.
     */
    private FtcTensorFlow.TargetInfo[] getDetectedTargetsInfo(String label, FtcTensorFlow.FilterTarget filter)
    {
        if (tensorFlow == null) throw new RuntimeException("TensorFlow Vision is not initialized!");

        FtcTensorFlow.TargetInfo[] detectedTargets = tensorFlow.getDetectedTargetsInfo(label, filter);
        if (detectedTargets != null)
        {
            Arrays.sort(detectedTargets, this::compareConfidence);
        }

        return detectedTargets;
    }   //getDetectedTargetsInfo

    /**
     * This method is called to validate the detected target as a duck.
     * To valid a valid duck, it must have:
     *  - correct aspect ratio
     *  - correct size
     *  - at expected location(s).
     *
     * @param target specifies the target to be validated.
     * @return true if target is valid, false if false positive.
     */
    private boolean validateDuck(Recognition target)
    {
        FtcTensorFlow.TargetInfo targetInfo = tensorFlow.getTargetInfo(target);
        double aspectRatio = (double)targetInfo.rect.width/(double)targetInfo.rect.height;
        double area = targetInfo.rect.width*targetInfo.rect.height;
        double distanceYTolerance = targetInfo.imageHeight/6.0;

        return targetInfo.label.equals(LABEL_DUCK) &&
               aspectRatio <= ASPECT_RATIO_TOLERANCE_UPPER &&
               aspectRatio >= ASPECT_RATIO_TOLERANCE_LOWER &&
               area <= TARGET_SIZE_TOLERANCE_UPPER &&
               area >= TARGET_SIZE_TOLERANCE_LOWER &&
               Math.abs(targetInfo.distanceFromCenter.y) <= distanceYTolerance;
    }   //validateDuck

    /**
     * This method determines the duck's barcode position 1, 2, or 3 (0 if no valid duck found).
     *
     * @param targetInfo specifies the detected duck's target info.
     * @return duck's barcode position.
     */
    private int determineDuckPosition(FtcTensorFlow.TargetInfo targetInfo)
    {
        int pos = 0;

        if (targetInfo != null)
        {
            double oneSixthImageWidth = targetInfo.imageWidth/6.0;

            if (targetInfo.distanceFromCenter.x <= -oneSixthImageWidth)
            {
                pos = 1;
            }
            else if (targetInfo.distanceFromCenter.x >= oneSixthImageWidth)
            {
                pos = 3;
            }
            else
            {
                pos = 2;
            }

            if (robot.blinkin != null)
            {
                // Turn off previous detection indication.
                robot.blinkin.setPatternState(duckPos1, false);
                robot.blinkin.setPatternState(duckPos2, false);
                robot.blinkin.setPatternState(duckPos3, false);

                switch (pos)
                {
                    case 1:
                        robot.blinkin.setPatternState(duckPos1, true);
                        break;

                    case 2:
                        robot.blinkin.setPatternState(duckPos2, true);
                        break;

                    case 3:
                        robot.blinkin.setPatternState(duckPos3, true);
                        break;
                }
            }
        }

        return pos;
    }   //determineDuckPosition

    /**
     * This method calls vision to detect all the ducks and returns their barcode positions in an array.
     *
     * @return duck barcode position array 1, 2, or 3, null if none found.
     */
    public int[] getCurrentDuckPositions()
    {
        int[] duckPositions = null;
        FtcTensorFlow.TargetInfo[] targetInfo =
            robot.vision.getDetectedTargetsInfo(Vision.LABEL_DUCK, robot.vision::validateDuck);

        if (targetInfo != null && targetInfo.length > 0)
        {
            duckPositions = new int[targetInfo.length];
            //
            // Target array is sorted with highest confidence first. Therefore, we process the array backward so
            // that the highest confidence target will be processed last. Therefore, the LED state will show the
            // highest confidence target position.
            //
            for (int i = targetInfo.length - 1; i >= 0; i--)
            {
                duckPositions[i] = robot.vision.determineDuckPosition(targetInfo[i]);
                // We don't have unlimited display lines, so only display the first three in the array.
                if (i < 3)
                {
                    robot.dashboard.displayPrintf(12 + i, "%s (Pos=%d)", targetInfo[i], duckPositions[i]);
                }
            }

            if (targetInfo.length < 3)
            {
                for (int i = targetInfo.length; i < 3; i++)
                {
                    robot.dashboard.displayPrintf(12 + i, "");
                }
            }

            lastDuckPositions = duckPositions;
        }

        return duckPositions;
    }   //getCurrentDuckPositions

    /**
     * This method returns the last detected duck barcode position.
     *
     * @return last detected duck barcode position, 0 if no duck detected.
     */
    public int getLastDuckPosition()
    {
        return lastDuckPositions != null? lastDuckPositions[0]: 0;
    }   //getLastDuckPosition

}   //class Vision
