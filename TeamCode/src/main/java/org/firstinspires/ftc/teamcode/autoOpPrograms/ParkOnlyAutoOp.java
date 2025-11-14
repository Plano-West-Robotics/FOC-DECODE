package org.firstinspires.ftc.teamcode.autoOpPrograms;

/**
 * Very bare-bones autonomous mode
 * Only parks the robo
 * does not use odometers
 * Literally only parks the robot
 */

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.NINETY;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.ONEEIGHTY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Parking Only Autonomous", group = "Autonomous")
public class ParkOnlyAutoOp extends OpMode {

    //Movement Motors
    Motor motorFL;
    Motor motorFR;
    Motor motorBL;
    Motor motorBR;

    @Override
    public void init()
    {
        this.motorFL = new Motor(hardwareMap,"motorFL");
        this.motorFR = new Motor(hardwareMap,"motorFR");
        this.motorBL = new Motor(hardwareMap,"motorBL");
        this.motorBR = new Motor(hardwareMap,"motorBR");
    }

    @Override
    public void start()
    {

    }

    @Override
    public void loop()
    {

    }

    private void moveTo(int x, int y)
    {

    }
}