package org.firstinspires.ftc.teamcode.teleopPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.subsystems.CameraSetup;
import org.firstinspires.ftc.teamcode.subsystems.Gear;

@TeleOp(name = "BackupTeleOp", group = "TeleOp")
public class BackupTeleOp extends OpMode {

    enum State
    {

    }

    public static final double IN_POW = 1;
    public static final double TRAN_POW = 1;

    public static final double OUT_POW = 1;

    Gamepad gp1;
    Gamepad gp2;
    Gamepad prevgp1;
    Gamepad prevgp2;

    Motor input;
    Motor tran;
    CRServo tranServo;
    Motor output;


    Motor fl;
    Motor fr;
    Motor bl;
    Motor br;

    //Gear gear;
    //CameraSetup camera;
    State state;

    @Override
    public void init() {

         fl = new Motor(hardwareMap, "fl");
         fr = new Motor(hardwareMap, "fr");
         bl = new Motor(hardwareMap, "bl");
         br = new Motor(hardwareMap, "br");

        input = new Motor(hardwareMap, "i");
        tran = new Motor(hardwareMap, "t");
        tranServo = hardwareMap.get(CRServo.class, "tServo");
        output = new Motor(hardwareMap, "o");
        //camera = new CameraSetup(hardwareMap);

        gp1 = new Gamepad();
        prevgp1 = new Gamepad();
        gp2 = new Gamepad();
        prevgp2 = new Gamepad();

        //gear = new Gear(hardwareMap, camera, input, tran, output);
        //gear.setAlliance(true);
    }

    @Override
    public void start()
    {
        //gear.startTrack();
    }

    @Override
    public void loop() {

        //camera.update();
        //gear.update();

        prevgp1.copy(gp1);
        gp1.copy(gamepad1);

        prevgp2.copy(gp2);
        gp2.copy(gamepad2);

        /*
             GP2 CONTROLS
             a - start intake
             b - end intake
             x - transfer on
             y - transfer off
         */

        if (gp2.a && !prevgp2.a)
            input.setPower(IN_POW);

        if (gp2.b && !prevgp2.b)
            input.setPower(0);

        if (gp2.x && !prevgp2.x)
        {
            tran.setPower(TRAN_POW);
            tranServo.setPower(TRAN_POW);
        }

        if (gp2.y && !prevgp2.y)
        {
            tran.setPower(0);
            tranServo.setPower(0);
        }

        if (gp2.left_bumper && !prevgp2.left_bumper)
            output.setPower(OUT_POW);

        if (gp2.right_bumper && !prevgp2.right_bumper)
            output.setPower(0);


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
    }
}
