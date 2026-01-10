package org.firstinspires.ftc.teamcode.teleopPrograms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.subsystems.LauncherTwo;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Driver Controlled")

public class DriverControlled extends OpMode {
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;
     Motor motorOUT;
     Motor motorIN;
     Motor motorTFER;

     LauncherTwo launcher2;


     Gamepad prevGp2;

    public void runOpMode(){

    }
    public void init(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");

        this.motorOUT = new Motor(hardwareMap, "o");
        this.motorIN = new Motor(hardwareMap, "i");
        this.motorTFER = new Motor(hardwareMap,"t"); //Just added this in case
        this.launcher2 = new LauncherTwo(motorIN, motorTFER, motorOUT);

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        motorIN.setDirection(DcMotor.Direction.REVERSE);

        this.prevGp2 = new Gamepad();


    }

    public void loop(){

        //forward backward turning
        fl.setPower(gamepad1.left_stick_y);
        fr.setPower(gamepad1.right_stick_y);
        bl.setPower(gamepad1.left_stick_y);
        br.setPower(gamepad1.right_stick_y);

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
        /*if(gamepad2.right_trigger > 0)
        {
            i.setPower(gamepad2.right_trigger);
        }
        if(gamepad2.left_trigger > 0)
        {
            i.setPower(-gamepad2.left_trigger);
        }*/

       if(gamepad2.a && !prevGp2.a){
            launcher2.inChange();
       }

        if(gamepad2.y && !prevGp2.y){
            launcher2.transferChange();
        }

        if(gamepad2.b && !prevGp2.b){
            launcher2.outChange();
        }


        prevGp2.copy(gamepad2);

    }
}