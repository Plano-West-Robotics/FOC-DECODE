package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;

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
    public static final double FLAP_CLOSED_POS = 1;
    public static final double FLAP_OPEN_POS = 0;

    //HARDWARE

    CameraSetup camera;
    Motor motorOUT;
    Servo servoAngle;

    //CONTROL VARIABLES

    public Launcher(CameraSetup camera, Motor flywheels, Servo servo)
    {
        this.servoAngle = servo;
        this.motorOUT = flywheels;
        this.camera = camera;

    }

    //

}
