package org.firstinspires.ftc.teamcode.tune;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tuningSubsystems.Gamepads;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.CameraSetup;
import org.firstinspires.ftc.teamcode.subsystems.Gear;

@TeleOp(name = "GearTester", group = "TeleOp")
public class GearTest extends OpMode {

    enum State
    {

    }

    Gamepad gp1;
    Gamepad prevgp1;

    Motor input;
    Motor tran;
    Motor output;


    Motor fl;
    Motor fr;
    Motor bl;
    Motor br;

    Gear gear;
    CameraSetup camera;
    State state;

    @Override
    public void init() {

         fl = new Motor(hardwareMap, "fl");
         fr = new Motor(hardwareMap, "fr");
         bl = new Motor(hardwareMap, "bl");
         br = new Motor(hardwareMap, "br");

        input = new Motor(hardwareMap, "i");
        tran = new Motor(hardwareMap, "t");
        output = new Motor(hardwareMap, "o");
        camera = new CameraSetup(hardwareMap);

        gp1 = new Gamepad();
        prevgp1 = new Gamepad();

        gear = new Gear(hardwareMap, camera, input, tran, output);
        gear.setAlliance(true);
    }

    @Override
    public void loop() {

        prevgp1.copy(gp1);
        gp1.copy(gamepad1);

        fl.setPower(-gamepad1.right_stick_y);
        fr.setPower(-gamepad1.left_stick_y);
        bl.setPower(-gamepad1.right_stick_y);
        br.setPower(-gamepad1.left_stick_y);

        if(gamepad1.left_trigger > 0 ){
            double leftStrafe = gamepad1.left_trigger;
            fl.setPower(-leftStrafe);
            fr.setPower(leftStrafe);
            bl.setPower(leftStrafe);
            br.setPower(-leftStrafe);
        }

        if(gamepad1.right_trigger > 0 ) {
            double rightStrafe = gamepad1.right_trigger;
            fl.setPower(rightStrafe);
            fr.setPower(-rightStrafe);
            bl.setPower(-rightStrafe);
            br.setPower(rightStrafe);
        }

        if (gp1.a && !prevgp1.a)
            gear.startTrack();

        if (gp1.b && !prevgp1.b)
            gear.finishTrack();

        camera.update();
        gear.update();
    }
}
