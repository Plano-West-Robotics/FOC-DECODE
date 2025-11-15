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
import org.firstinspires.ftc.teamcode.examplePedroAuto.BasicStates;
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

    //Pedro Pathing Instance Variables
    private Follower follower;

    //These values can be adjusted later depending on time
    private Pose startPose = new Pose(56,8);

    private Pose connection = new Pose(76,46);

    private Pose parkPose = new Pose(38.718, 33.934);

    private Path parkPath;

    //Sets up Timer
    private Timer autoTimer;
    private static double MAX_TIME = 30.0;


    /**Initializes the motors, follower, path, and timer
     *
     */
    @Override
    public void init()
    {
        this.motorFL = new Motor(hardwareMap,"motorFL");
        this.motorFR = new Motor(hardwareMap,"motorFR");
        this.motorBL = new Motor(hardwareMap,"motorBL");
        this.motorBR = new Motor(hardwareMap,"motorBR");

        this.follower = Constants.createFollower(hardwareMap);
        buildTheParkingPath();
        this.follower.setStartingPose(startPose);

        autoTimer = new Timer();



    }

    /**Resets the timer
     *
     */
    @Override
    public void start()
    {
        autoTimer.resetTimer();

    }

    /**Continues this code until time either runs out or the robot is in position
     *
     */
    @Override
    public void loop()
    {
        if(autoTimer.getElapsedTimeSeconds() >= MAX_TIME){
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
        if(follower.isBusy()){
            follower.update();
        }
        else{
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
    }

    /**Builds the parking path based on values found
     *
     */
    private void buildTheParkingPath(){
        parkPath = new Path(new BezierCurve(startPose, connection, parkPose));
        parkPath.setLinearHeadingInterpolation(0, NINETY);
    }
}