package Ftc2022FreightFrenzy_3543;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcIntake;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;

@TeleOp(name="FtcTestOp", group="Ftc3543")
public class TestOpMode extends FtcOpMode
{
    FtcDashboard dashboard;
    FtcGamepad gamepad;
    TrcIntake intake;
    String intakeOwner = null;

    @Override
    public void initRobot()
    {
        dashboard = FtcDashboard.getInstance();
        gamepad = new FtcGamepad("GamePad", gamepad1, this::gamepadButtonEvent);
        gamepad.setYInverted(true);
        TrcIntake.Parameters intakeParams = new TrcIntake.Parameters()
            .setMotorInverted(true)
            .setTriggerInverted(true)
            .setAnalogThreshold(4.0)
            .setMsgTracer(TrcDbgTrace.getGlobalTracer());
        intake = new Intake("intake", intakeParams).getIntake();
    }   //initRobot

    @Override
    public void runPeriodic(double elapsedTime)
    {
        if (intakeOwner == null)
        {
            intake.setPower(gamepad.getRightStickY(true));
        }
        dashboard.displayPrintf(1, "Distance: %.3f cm", intake.getSensorValue());
    }   //runPeriodic

    public void gamepadButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        final String ownerID = "autoAssistPickup";

        dashboard.displayPrintf(
            7, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");
        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                if (pressed && intake.acquireExclusiveAccess(ownerID))
                {
                    intakeOwner = ownerID;
                    intake.autoAssist(ownerID, RobotParams.INTAKE_POWER_PICKUP, null, this::intakeCompletion, 5.0);
                }
                break;

            case FtcGamepad.GAMEPAD_B:
                if (pressed && intake.acquireExclusiveAccess(ownerID))
                {
                    intakeOwner = ownerID;
                    intake.autoAssist(ownerID, RobotParams.INTAKE_POWER_DUMP, null, this::intakeCompletion, 5.0);
                }
                break;

            case FtcGamepad.GAMEPAD_X:
                break;

            case FtcGamepad.GAMEPAD_Y:
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                break;
        }
    }   //gamepadButtonEvent

    private void intakeCompletion(Object context)
    {
        intake.releaseExclusiveAccess(intakeOwner);
        intakeOwner = null;
    }   //intakeCompletion

}   //class TestOpMode
