package org.firstinspires.ftc.teamcode.tune;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.tuningSubsystems.Analog;
import org.firstinspires.ftc.teamcode.tuningSubsystems.Button;
import org.firstinspires.ftc.teamcode.tuningSubsystems.Gamepads;

import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "GamepadTester", group = "Testing")
public class GamepadsTest extends OpMode
{
    public Gamepads gamepads;

    @Override
    public void init()
    {
        gamepads = new Gamepads(gamepad1, gamepad2);

        // Uncomment if you want telemetry to show up on FTCDashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop()
    {
        for (Analog analogValue : Analog.values())
        {
            telemetry.addData(analogValue.name(), gamepads.getAnalogValue(analogValue));
        }

        for (Button buttonValue : Button.values())
        {
            telemetry.addData(buttonValue.name(), gamepads.isPressed(buttonValue));
        }

        telemetry.update();
        gamepads.update(gamepad1, gamepad2);
    }
}