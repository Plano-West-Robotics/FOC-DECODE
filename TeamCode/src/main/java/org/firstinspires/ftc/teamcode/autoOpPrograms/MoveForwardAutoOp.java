package org.firstinspires.ftc.teamcode.autoOpPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Motor;

@Autonomous(name = "Move Forward Autonomous", group = "Autonomous")
public class MoveForwardAutoOp extends LinearOpMode
{
    public Motor fl, fr, bl, br;

    @Override
    public void runOpMode() throws InterruptedException
    {
        fl = new Motor(hardwareMap, "fl");
        fr = new Motor(hardwareMap, "fr");
        bl = new Motor(hardwareMap, "bl");
        br = new Motor(hardwareMap, "br");

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.reverse();
        bl.reverse();

        waitForStart();

        if (isStarted())
        {
            fl.setPower(0.1);
            fr.setPower(0.1);
            bl.setPower(0.1);
            br.setPower(0.1);

            sleep(1000);

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
    }
}
