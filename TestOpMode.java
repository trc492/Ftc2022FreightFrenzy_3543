package Ftc2022FreightFrenzy_3543;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcGameController;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;

@TeleOp(name="FtcTestOp", group="Ftc3543")
public class TestOpMode extends FtcOpMode
{
    FtcDashboard dashboard;
    FtcGamepad gamepad;
    Intake intake;
    String intakeOwner = null;

    @Override
    public void initRobot()
    {
        dashboard = FtcDashboard.getInstance();
        gamepad = new FtcGamepad("GamePad", gamepad1, this::gamepadButtonEvent);
        gamepad.setYInverted(true);
        intake = new Intake("intakeMotor");
    }   //initRobot

    @Override
    public void runPeriodic(double elapsedTime)
    {
        if (intakeOwner == null)
        {
            intake.set(gamepad.getRightStickY(true));
        }
        dashboard.displayPrintf(1, "Distance: %.3f cm", intake.getDistance());
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
                    intake.pickupFreight(ownerID, -1.0, null, this::intakeCompletion, 10.0);
                }
                break;

            case FtcGamepad.GAMEPAD_B:
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
