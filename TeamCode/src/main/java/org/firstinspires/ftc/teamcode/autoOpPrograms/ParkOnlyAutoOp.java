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
import org.firstinspires.ftc.teamcode.myAuto.Practice1;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Parking Only Autonomous", group = "Autonomous")
public class ParkOnlyAutoOp extends OpMode {

    public enum States
    {
        INIT,
        FIRST_MOVE,
        SECOND_MOVE,
        END
    }

    //Movement Motors
    Motor motorFL;
    Motor motorFR;
    Motor motorBL;
    Motor motorBR;

    //Sets up Timer
    private static final double MOVE_TIME = 0.5;
    private Timer autoTimer;
    private Timer moveTimer;
    private static double MAX_TIME = 25.0;

    States state;


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

        moveTimer = new Timer();
        autoTimer = new Timer();

        state = States.INIT;

    }

    /**Resets the timer
     *
     */
    @Override
    public void start()
    {
        autoTimer.resetTimer();
        moveTimer.resetTimer();
    }

    /**Continues this code until time either runs out or the robot is in position
     *
     */
    @Override
    public void loop()
    {
        switch(state)
        {
            case INIT:
                moveTimer.resetTimer();
                state = States.FIRST_MOVE;
                break;
            case FIRST_MOVE:
                motorFL.setPower(1);
                motorFR.setPower(1);
                motorBL.setPower(1);
                motorBR.setPower(1);

                if (moveTimer.getElapsedTimeSeconds() >= 1) {
                    moveTimer.resetTimer();
                    state = States.SECOND_MOVE;
                }

                break;
            case SECOND_MOVE:
                motorFL.setPower(1);
                motorFR.setPower(-1);
                motorBL.setPower(-1);
                motorBR.setPower(1);

                if (moveTimer.getElapsedTimeSeconds() >= 1.5)
                {
                    state = States.END;
                }

                break;
            case END:
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                break;
            default:
                System.out.println("Undefined state reached");
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                break;
        }
    }

    /**Builds the parking path based on values found
     *
     */

}