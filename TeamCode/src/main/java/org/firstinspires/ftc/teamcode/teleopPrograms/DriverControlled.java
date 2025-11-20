package org.firstinspires.ftc.teamcode.teleopPrograms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Driver Controlled")

public class DriverControlled extends OpMode {
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;

    private CRServo servoLoader;

    private CRServo servoLoader2;
    private DcMotor i;
    private DcMotor o;

    public void runOpMode() {

    }

    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");


        servoLoader = hardwareMap.get(CRServo.class, "servoLoader");
        servoLoader2 = hardwareMap.get(CRServo.class, "servoLoader2");
        i = hardwareMap.get(DcMotor.class, "i");
        o = hardwareMap.get(DcMotor.class, "o");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        i.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {

        //forward backward turning
        /*if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
            fl.setPower(gamepad1.left_stick_y);
            bl.setPower(gamepad1.left_stick_y);
        }

        if(gamepad1.right_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
            fr.setPower(gamepad1.right_stick_y);
            br.setPower(gamepad1.right_stick_y);
        }*/
        fl.setPower(gamepad1.left_stick_y);
        fr.setPower(gamepad1.right_stick_y);
        bl.setPower(gamepad1.left_stick_y);
        br.setPower(gamepad1.right_stick_y);


        //strafing
        if (gamepad1.dpad_left) {
            fl.setPower(0.5);
            fr.setPower(-0.5);
            bl.setPower(-0.5);
            br.setPower(0.5);
        }
        if (gamepad1.dpad_right) {
            fl.setPower(-0.5);
            fr.setPower(0.5);
            bl.setPower(0.5);
            br.setPower(-0.5);

        }

        /*if (gamepad1.x) {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
        if (gamepad1.a) {
            servoLoader.setPower(-1);
            servoLoader2.setPower(-1);
        }

        if (gamepad1.b) {
            servoLoader.setPower(0);
            servoLoader2.setPower(0);
        }
        if (gamepad1.x) {
            i.setPower(-1);
        }
        if (gamepad1.y) {
            i.setPower(1);
        }
        if (gamepad1.left_trigger > 0) {
            o.setPower(-gamepad1.left_trigger);
        }

        if (gamepad1.left_trigger == 0) {
            o.setPower(0);
        }*/
    }
}