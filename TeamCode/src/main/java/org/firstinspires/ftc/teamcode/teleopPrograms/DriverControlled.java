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

        servoLoader = hardwareMap.get(CRServo.class, "servo");
        i = hardwareMap.get(DcMotor.class, "intake");
        o = hardwareMap.get(DcMotor.class, "transport");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop(){

        //forward backward turning
        fl.setPower(gamepad1.left_stick_y);
        fr.setPower(gamepad1.right_stick_y);
        bl.setPower(gamepad1.left_stick_y);
        br.setPower(gamepad1.right_stick_y);

        //strafing
        if(gamepad1.left_bumper) {
            fl.setPower(gamepad1.left_trigger);
            fr.setPower(-gamepad1.left_trigger);
            bl.setPower(-gamepad1.left_trigger);
            br.setPower(gamepad1.left_trigger);
        }
        if(gamepad1.right_bumper) {
            fl.setPower(-gamepad1.right_trigger);
            fr.setPower(gamepad1.right_trigger);
            bl.setPower(gamepad1.right_trigger);
            br.setPower(-gamepad1.right_trigger);
        }

        //arm manual
        //arm1.setPower(gamepad2.right_stick_y);
        //arm2.setPower(gamepad2.left_stick_y);

        //arm in sync
        /*if(gamepad2.dpad_up){
            arm1.setPower(-1);
            arm2.setPower(-1);
        }
        if(gamepad2.dpad_down){
            arm1.setPower(1);
            arm2.setPower(1);
        }*/
        if(gamepad1.a)
        {
            servoLoader.setPower(1);
        }
        if(gamepad1.b)
        {
            servoLoader.setPower(0);
        }
        if(gamepad1.right_trigger > 0)
        {
            i.setPower(gamepad1.right_trigger);
        }
        if(gamepad1.left_trigger > 0)
        {
            o.setPower(gamepad1.left_trigger);
        }
        //intake
        /*if(gamepad2.right_trigger > 0){
            intake.setPower(gamepad2.right_trigger+1000);
        }
        if(gamepad2.left_trigger > 0){
            intake.setPower(-gamepad2.left_trigger-1000);
        }
        if(gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0){
            intake.setPower(0);
        }

        //hanging
        if (gamepad2.y) {
            while (!gamepad2.a){
                runTime.reset();
                arm1.setPower(10000);
                arm2.setPower(10000);
                if (runTime.time() >= 100){
                    arm1.setPower(0);
                    arm2.setPower(0);
                }
            }
        }
        if (gamepad2.a){
            while (!gamepad2.y){
                arm1.setPower(0);
                arm2.setPower(0);
            }
        }*/
    }
}