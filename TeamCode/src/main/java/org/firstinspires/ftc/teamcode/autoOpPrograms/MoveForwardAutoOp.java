package org.firstinspires.ftc.teamcode.autoOpPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Motor;

@Autonomous(name = "Move Forward Autonomous", group = "Autonomous")
public class MoveForwardAutoOp extends LinearOpMode
{
    public Motor fl, fr, bl, br, o, i, t;

    public void shootBalls() {
        //rotate towards goal
        fl.setPower(-0.1);
        fr.setPower(0.1);
        bl.setPower(-0.1);
        br.setPower(0.1);

        sleep(500);

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        //start outtake
        o.setPower(0.8);
        sleep(7000);
        //start transfer to shoot first ball
        t.setPower(1);

        sleep(5000);

        //start intake to shoot second ball
        i.setPower(0.3);

        sleep(5000);

        //stop launcher
        o.setPower(0);
        i.setPower(0);
        t.setPower(0);

        //rotate back to facing forward
        fl.setPower(0.1);
        fr.setPower(-0.1);
        bl.setPower(0.1);
        br.setPower(-0.1);

        sleep(500);

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        fl = new Motor(hardwareMap, "fl");
        fr = new Motor(hardwareMap, "fr");
        bl = new Motor(hardwareMap, "bl");
        br = new Motor(hardwareMap, "br");

        o = new Motor(hardwareMap, "o");
        i = new Motor(hardwareMap, "i");
        t = new Motor(hardwareMap, "t");

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.reverse();
        bl.reverse();

        waitForStart();

        if (isStarted())
        {
            //move away from wall
            fl.setPower(0.3);
            fr.setPower(0.3);
            bl.setPower(0.3);
            br.setPower(0.3);

            sleep(400);

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            sleep(500);

            //strafe left
            fl.setPower(0.3);
            fr.setPower(-0.3);
            bl.setPower(-0.3);
            br.setPower(0.3);

            sleep(500);

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            //rotate and shoot both balls
            //shootBalls();

        }
    }
}
