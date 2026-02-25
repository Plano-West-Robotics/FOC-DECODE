package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Launcher controls the ball-launching mechanisms for the robot
 * Mechanisms Include:
 * Flywheels that launch the balls
 * Also controls the turret angle adjuster.
 * UPDATED 2/23/2026
 */
public class Launcher {

    //CLASS CONSTANTS
    public static final double MAX_FLYWHEEL_POWER = 1.0/5.0;

    private static final double MAX_ANGLE = 60;
    private static final double TRUE_MAX_ANGLE = 180;

    private static final double GRAV = 9.8;
    private static final double GOAL_HEIGHT = 15;
    private static final double ROBOT_HEIGHT = 4.85;

    //HARDWARE

    private CameraSetup camera;
    private Motor motorOUT;
    private Servo servoAngle;

    //CONTROL VARIABLES

    public Launcher(CameraSetup camera, Motor flywheels, Servo servo)
    {
        this.servoAngle = servo;
        this.motorOUT = flywheels;
        this.camera = camera;

    }

    public void lock(AprilTagDetection tag)
    {
        double dist = tag.ftcPose.range;
        if(dist > -1) //SET TO -1 SO THAT IT ONLY EVER CHANGES VELOCITY
        {
            velocityChange(dist);
        }
        else{
            angleChange(dist);
        }
    }

    public double velocityChange(double dist)
    {
        double outPower = (MAX_ANGLE / ( 2 * Math.PI * RADIUS));
        outPower *= (2 * GRAV * Math.pow(dist, 2));
        outPower /= (Math.tan(MAX_ANGLE) * dist) - (GOAL_HEIGHT-ROBOT_HEIGHT);

        return outPower;
    }

    public double angleChange(double dist)
    {
        double velSquared = Math.pow(MAX_VELOCITY, 2); //replace 20
        double distSquared = Math.pow(dist, 2);

        double root = (GOAL_HEIGHT-ROBOT_HEIGHT) + ((GRAV * distSquared) / (2 * velSquared));
        root *= ((2 * GRAV * distSquared) / velSquared);
        root = Math.sqrt(distSquared - root);

        double angle = velSquared * (dist + root);
        angle /= (GRAV * distSquared);
        angle = Math.atan(angle);

        return angle;

    }


}
