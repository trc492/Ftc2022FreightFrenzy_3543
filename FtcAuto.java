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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import TrcCommonLib.command.CmdPidDrive;
import TrcCommonLib.command.CmdTimedDrive;

import java.util.Locale;

import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcChoiceMenu;
import TrcFtcLib.ftclib.FtcMatchInfo;
import TrcFtcLib.ftclib.FtcMenu;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcValueMenu;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name="FtcAutonomous", group="Ftc3543")
public class FtcAuto extends FtcOpMode
{
    public enum AutoStrategy
    {
        AUTO_NEAR_CAROUSEL,
        AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_STORAGE_PARKING,
        AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_WAREHOUSE_PARKING,
        AUTO_FAR_CAROUSEL,
        AUTO_SHUTTLE_BACK_AND_FORTH,
        PID_DRIVE,
        TIMED_DRIVE,
        AUTO_TEST,
        DO_NOTHING
    }   //enum AutoStrategy

    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum Parking
    {
        NO_PARKING,
        STORAGE_PARKING,
        WAREHOUSE_PARKING,
    }   //enum Parking

    /**
     * This class stores the autonomous menu choices.
     */
    public static class AutoChoices
    {
        double startDelay = 0.0;
        Alliance alliance = Alliance.RED_ALLIANCE;
        AutoStrategy strategy = AutoStrategy.AUTO_NEAR_CAROUSEL;
        boolean freightDelivery = false;
        boolean duckDelivery = false;
        boolean doCarousel = false;
        Parking parking = Parking.WAREHOUSE_PARKING;
        double xTarget = 0.0;
        double yTarget = 0.0;
        double turnTarget = 0.0;
        double driveTime = 0.0;
        double drivePower = 0.0;

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "startDelay=%.0f " +
                "alliance=\"%s\" " +
                "strategy=\"%s\" " +
                "freightDelivery=\"%s\" " +
                "duckDelivery=\"%s\" " +
                "doCarousel=\"%s\" " +
                "parking=\"%s\" " +
                "xTarget=%.1f " +
                "yTarget=%.1f " +
                "turnTarget=%.0f " +
                "driveTime=%.0f " +
                "drivePower=%.1f",
                startDelay, alliance, strategy, freightDelivery, duckDelivery, doCarousel, parking,
                xTarget, yTarget, turnTarget, driveTime, drivePower);
        }   //toString

    }   //class AutoChoices

    private static final String moduleName = "FtcAuto";
    private static final boolean logEvents = true;
    private static final boolean debugPid = true;

    private Robot robot;
    private FtcMatchInfo matchInfo;
    private final AutoChoices autoChoices = new AutoChoices();
    private TrcRobot.RobotCommand autoCommand = null;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void initRobot()
    {
        final String funcName = "initRobot";
        //
        // Create and initialize robot object.
        //
        robot = new Robot(TrcRobot.getRunMode());
        //
        // Open trace log.
        //
        if (RobotParams.Preferences.useTraceLog)
        {
            matchInfo = FtcMatchInfo.getMatchInfo();
            String filePrefix = String.format(Locale.US, "%s%02d", matchInfo.matchType, matchInfo.matchNumber);
            robot.globalTracer.openTraceLog(RobotParams.LOG_PATH_FOLDER, filePrefix);
        }
        //
        // Create and run choice menus.
        //
        doAutoChoicesMenus();
        //
        // Create autonomous command according to chosen strategy.
        //
        switch (autoChoices.strategy)
        {
            case AUTO_NEAR_CAROUSEL:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdAutoNearCarousel(robot, autoChoices);
                }
                break;
            case AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_STORAGE_PARKING:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdAutoNearCarousel(robot, autoChoices);
                }
                break;
            case AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_WAREHOUSE_PARKING:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdAutoNearCarousel(robot, autoChoices);
                }
                break;

            case AUTO_FAR_CAROUSEL:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdAutoFarCarousel(robot, autoChoices);
                }
                break;

            case AUTO_SHUTTLE_BACK_AND_FORTH:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdAutoShuttleBackAndForth(robot, autoChoices);
                }
                break;

            case PID_DRIVE:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, autoChoices.startDelay,
                        autoChoices.drivePower, null,
                        new TrcPose2D(autoChoices.xTarget*12.0, autoChoices.yTarget*12.0, autoChoices.turnTarget));
                }
                break;

            case TIMED_DRIVE:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, autoChoices.startDelay, autoChoices.driveTime,
                        0.0, autoChoices.drivePower, 0.0);
                }
                break;

            case AUTO_TEST:
                if (!RobotParams.Preferences.noRobot)
                {
                    autoCommand = new CmdAutoTest(robot);
                }
                break;

            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }

        if (robot.vision != null)
        {
            if (RobotParams.Preferences.useVuforia)
            {
                robot.globalTracer.traceInfo(funcName, "Enabling Vuforia.");
                robot.vision.setVuforiaEnabled(true);
            }

            if (RobotParams.Preferences.useTensorFlow)
            {
                robot.globalTracer.traceInfo(funcName, "Enabling TensorFlow.");
                robot.vision.setTensorFlowEnabled(true);
            }
        }
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called periodically after initRobot() is called but before competition starts. For this season,
     * we are detecting the duck's barcode position before the match starts.
     */
    @Override
    public void initPeriodic()
    {
        if (robot.vision != null && RobotParams.Preferences.useTensorFlow)
        {
            robot.vision.getBestDuckPosition();
        }
    }   //initPeriodic

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.dashboard.clearDisplay();

        if (RobotParams.Preferences.useTraceLog)
        {
            robot.globalTracer.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(moduleName, "***** Starting autonomous *****");
        if (matchInfo != null)
        {
            robot.globalTracer.logInfo(moduleName, "MatchInfo", "%s", matchInfo);
        }
        robot.globalTracer.logInfo(moduleName, "AutoChoices", "%s", autoChoices);
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(true);
        }

        robot.robotDrive.pidDrive.setMsgTracer(robot.globalTracer, logEvents, debugPid);
        robot.robotDrive.purePursuitDrive.setMsgTracer(robot.globalTracer, logEvents, debugPid);
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // Opmode is about to stop, cancel autonomous command in progress if any.
        //
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(false);
        }

        printPerformanceMetrics(robot.globalTracer);

        if (robot.globalTracer.tracerLogIsOpened())
        {
            robot.globalTracer.closeTraceLog();
        }
    }   //stopMode

    /**
     * This method is called periodically as fast as the control system allows. Typically, you put code that requires
     * servicing at a higher frequency here. To make the robot as responsive and as accurate as possible especially
     * in autonomous mode, you will typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            //
            // Run the autonomous command.
            //
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //runContinuous

    /**
     * This method creates the autonomous menus, displays them and stores the choices.
     */
    private void doAutoChoicesMenus()
    {
        //
        // Construct menus.
        //
        FtcValueMenu startDelayMenu = new FtcValueMenu(
            "Start delay time:", null, 0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", startDelayMenu);
        FtcChoiceMenu<AutoStrategy> strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", allianceMenu);
        FtcChoiceMenu<Boolean> freightDeliveryMenu = new FtcChoiceMenu<>("Freight Delivery:", strategyMenu);
        FtcChoiceMenu<Boolean> duckDeliveryMenu = new FtcChoiceMenu<>("Duck Delivery", freightDeliveryMenu);
        FtcChoiceMenu<Boolean>carouselMenu=new FtcChoiceMenu<>("Carousel:", duckDeliveryMenu);
        FtcChoiceMenu<Parking>parkingMenu=new FtcChoiceMenu<>("Parking:", carouselMenu);

        FtcValueMenu xTargetMenu = new FtcValueMenu(
            "xTarget:", strategyMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu yTargetMenu = new FtcValueMenu(
            "yTarget:", xTargetMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu turnTargetMenu = new FtcValueMenu(
            "turnTarget:", yTargetMenu, -180.0, 180.0, 5.0, 90.0, " %.0f ft");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
            "Drive time:", strategyMenu, 0.0, 30.0, 1.0, 5.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
            "Drive power:", strategyMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");

        startDelayMenu.setChildMenu(allianceMenu);
        xTargetMenu.setChildMenu(yTargetMenu);
        yTargetMenu.setChildMenu(turnTargetMenu);
        turnTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        //
        // Populate choice menus.
        //
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, strategyMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, strategyMenu);

        strategyMenu.addChoice("Near Carousel Autonomous", AutoStrategy.AUTO_NEAR_CAROUSEL, true, freightDeliveryMenu);
        strategyMenu.addChoice("Far Carousel Autonomous", AutoStrategy.AUTO_FAR_CAROUSEL, false, freightDeliveryMenu);
        //the cmd shuttle back and forth and near carousel with duck does not require choices
        strategyMenu.addChoice("Shuttle Back and Forth Autonomous", AutoStrategy.AUTO_SHUTTLE_BACK_AND_FORTH, false);
        strategyMenu.addChoice("Near Carousel With Duck Delivery Parking Storage Unit Auto", AutoStrategy.AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_STORAGE_PARKING, false);
        strategyMenu.addChoice("Near Carousel With Duck Delivery Parking Warehouse Auto", AutoStrategy.AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_WAREHOUSE_PARKING, false);

        strategyMenu.addChoice("PID Drive", AutoStrategy.PID_DRIVE, false, xTargetMenu);
        strategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Auto Test", AutoStrategy.AUTO_TEST, false);
        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING, false);

        freightDeliveryMenu.addChoice("Do Delivery", true, true, duckDeliveryMenu);
        freightDeliveryMenu.addChoice("No Delivery", false, false, duckDeliveryMenu);

        duckDeliveryMenu.addChoice("Do Duck Delivery ", true, false, carouselMenu);
        duckDeliveryMenu.addChoice("No Duck Delivery", false, true, carouselMenu);

        carouselMenu.addChoice("Do Carousel", true, true, parkingMenu);
        carouselMenu.addChoice("No Carousel", false, false, parkingMenu);

        parkingMenu.addChoice("Storage Parking", Parking.STORAGE_PARKING, true);
        parkingMenu.addChoice("Warehouse Parking", Parking.WAREHOUSE_PARKING, false);
        parkingMenu.addChoice("No Parking", Parking.NO_PARKING, false);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(startDelayMenu);
        //
        // Fetch choices.
        //
        autoChoices.startDelay = startDelayMenu.getCurrentValue();
        autoChoices.alliance = allianceMenu.getCurrentChoiceObject();
        autoChoices.strategy = strategyMenu.getCurrentChoiceObject();
        autoChoices.freightDelivery = freightDeliveryMenu.getCurrentChoiceObject();
        autoChoices.duckDelivery = duckDeliveryMenu.getCurrentChoiceObject();
        autoChoices.doCarousel = carouselMenu.getCurrentChoiceObject();
        autoChoices.parking = parkingMenu.getCurrentChoiceObject();
        autoChoices.xTarget = xTargetMenu.getCurrentValue();
        autoChoices.yTarget = yTargetMenu.getCurrentValue();
        autoChoices.turnTarget = turnTargetMenu.getCurrentValue();
        autoChoices.driveTime = driveTimeMenu.getCurrentValue();
        autoChoices.drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        //if we are doing nearcarousel duck delivery and storage or warehouse, we preset all of the choices
        if(autoChoices.strategy==AutoStrategy.AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_STORAGE_PARKING||autoChoices.strategy==AutoStrategy.AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_STORAGE_PARKING){
            autoChoices.freightDelivery = true;
            autoChoices.duckDelivery = true;
            autoChoices.doCarousel = true;
            autoChoices.parking = Parking.STORAGE_PARKING;
            if(autoChoices.strategy == AutoStrategy.AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_WAREHOUSE_PARKING){
                autoChoices.parking = Parking.WAREHOUSE_PARKING;
            }
        }
        robot.dashboard.displayPrintf(2, "Auto Choices: %s", autoChoices);
    }   //doAutoChoicesMenus

}   //class FtcAuto
