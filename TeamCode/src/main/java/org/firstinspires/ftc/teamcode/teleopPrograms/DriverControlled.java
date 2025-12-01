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
    private DcMotor i;
    private DcMotor o;

    public void runOpMode(){

    }
    public void init(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");

        servoLoader = hardwareMap.get(CRServo.class, "servoLoader");
        i = hardwareMap.get(DcMotor.class, "i");
        o = hardwareMap.get(DcMotor.class, "o");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        i.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop(){

        //forward backward turning
        fl.setPower(gamepad1.left_stick_y);
        fr.setPower(gamepad1.right_stick_y);
        bl.setPower(gamepad1.left_stick_y);
        br.setPower(gamepad1.right_stick_y);

        //strafing
        /*if(gamepad1.dpad_left) {
            fl.setPower(gamepad1.left_trigger);
            fr.setPower(-gamepad1.left_trigger);
            bl.setPower(-gamepad1.left_trigger);
            br.setPower(gamepad1.left_trigger);
        }
        if(gamepad1.dpad_right) {
            fl.setPower(-gamepad1.right_trigger);
            fr.setPower(gamepad1.right_trigger);
            bl.setPower(gamepad1.right_trigger);
            br.setPower(-gamepad1.right_trigger);
        }*/
        if(gamepad1.left_trigger > 0 ){
            double leftStrafe = gamepad1.left_trigger;
            fl.setPower(leftStrafe);
            fr.setPower(-leftStrafe);
            bl.setPower(-leftStrafe);
            br.setPower(leftStrafe);
        }
        if(gamepad1.right_trigger > 0 ){
            double rightStrafe = gamepad1.right_trigger;
            fl.setPower(-rightStrafe);
            fr.setPower(rightStrafe);
            bl.setPower(rightStrafe);
            br.setPower(-rightStrafe);
        }

       /* if(gamepad1.a)
        {
            servoLoader.setPower(-1);
        }
        if(gamepad1.b)
        {
            servoLoader.setPower(0);
        }
       /* if(gamepad1.right_trigger > 0)
        {
            i.setPower(-gamepad1.right_trigger);
        }
        if(gamepad1.right_trigger == 0){
            i.setPower(0);
        }
        if(gamepad1.left_trigger > 0) {
            o.setPower(-gamepad1.left_trigger);
        }
        if(gamepad1.left_trigger == 0){
            o.setPower(0);
        }*/
    }
}