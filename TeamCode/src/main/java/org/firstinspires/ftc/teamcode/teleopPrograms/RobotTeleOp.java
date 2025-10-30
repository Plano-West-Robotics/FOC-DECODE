package org.firstinspires.ftc.teamcode.teleopPrograms;

/**
 * Working TeleOp Control program for the FOC Robot
 * Should have working Field-Centric and Robot-Centric Control mode
 *
 * @version 1.0.0.0
 * @date 10/27/2025
 */

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.NINETY;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.ONEEIGHTY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "TeleOp v1.0.0.0", group = "Main")
public class RobotTeleOp extends OpMode
{
    //CONSTANTS
    public static final double MAX_FLYWHEEL_POWER = 1.0/3.0;

    //ENUMS

    //Used to determine which movement mode the robot is in
    private enum DriveMode
    {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    //INSTANCE VARIABLES

    //Declaring Misc Hardware
    IMU imu;

    //Declaring Motors
    //Movement Motors
    Motor motorFL;
    Motor motorFR;
    Motor motorBL;
    Motor motorBR;

    //Flywheel Motors
    Motor motorLFW;
    Motor motorRFW;

    //Declaring Servos
    //CURRENTLY UNUSED
    Servo servo1;
    Servo servo2;

    //Declaring GamePads
    //Gamepad 1 controls Movement; Gamepad 2 controls something else.
    Gamepad gp1;
    Gamepad gp2;
    Gamepad prevGp1;
    Gamepad prevGp2;

    //Declaring State controllers && Boolean logic controllers
    //State variables
    DriveMode movementMode;

    //Boolean variables
    boolean flywheelsOn;

    @Override
    public void init() {
        //Declaring hardware parts + gamepads
        this.motorFL = new Motor(hardwareMap,"motorFL");
        this.motorFR = new Motor(hardwareMap,"motorFR");
        this.motorBL = new Motor(hardwareMap,"motorBL");
        this.motorBR = new Motor(hardwareMap,"motorBR");

        //Code for theoretical flywheels
        this.motorLFW = new Motor(hardwareMap, "motorLFW");
        this.motorRFW = new Motor(hardwareMap, "motorRFW");

        //Reversing Right motors.
        this.motorFR.reverse();
        this.motorBR.reverse();

        //Servos aren't currently on the robot and thus are commented out.

        /*
        this.servo1 = hardwareMap.get(Servo.class, "servo1");
        this.servo2 = hardwareMap.get(Servo.class, "servo2");
        */

        this.imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        this.imu.initialize(parameters);


        this.gp1 = new Gamepad();
        this.gp2 = new Gamepad();
        this.prevGp1 = new Gamepad();
        this.prevGp2 = new Gamepad();

        //Declaring starting enum states
        this.movementMode = DriveMode.FIELD_CENTRIC;

        this.flywheelsOn = false;

    }

    @Override
    public void loop() {
        prevGp1.copy(gp1);
        prevGp2.copy(gp2);
        gp1.copy(gamepad1);
        gp2.copy(gamepad2);

        //Movement Handler
        if (movementMode.equals(DriveMode.ROBOT_CENTRIC))
            robotCentricMovement();
        else
            fieldCentricMovement();

        //State Machine Handler
        /** ---------------
         * GAMEPAD 1 CONTROL
         * Left Stick - Directional Movement
         * Right Stick - Rotational Movement
         * A - Field Centric Drive Mode
         * B - Robot Centric Drive Mode
         * X - Toggles flywheels (launching mechanism)
         */
        if (gp1.a && !prevGp1.a)
            movementMode = DriveMode.FIELD_CENTRIC;
        else if (gp1.b && !prevGp1.b)
            movementMode = DriveMode.ROBOT_CENTRIC;
        if (gp1.x && !prevGp1.x)
            changeFlywheelState();

    }

    /**
     * toggles whether or not the flywheels are on
     * also updates boolean variable controlling flywheels
     */
    private void changeFlywheelState()
    {
        flywheelsOn = !flywheelsOn;
        double leftPower;
        double rightPower;

        if (flywheelsOn)
        {
            leftPower = MAX_FLYWHEEL_POWER;
            rightPower = -MAX_FLYWHEEL_POWER;
        }
        else
        {
            leftPower = 0;
            rightPower = 0;
        }

        motorLFW.setPower(leftPower);
        motorRFW.setPower(rightPower);
    }

    /**
     * Moves the robot based on the orientation of the FIELD
     * Left joystick moves the robot
     * Right joystick rotates the robot
     * Method also uninverts vertical joystick controls (Up moves up)
     * Movement is handled before rotation of the robot occurs
     *
     * REMEMBER GP1 CONTROLS MOVEMENT AND ROTATION
     */
    private void fieldCentricMovement()
    {
        double moveY = -gp1.left_stick_y;
        double moveX = gp1.left_stick_x;
        double rotate = gp1.right_stick_x;

        //Calculate robot centric power first
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = moveX * Math.cos(-botHeading) - moveY * Math.sin(-botHeading);
        double rotY = moveX * Math.sin(-botHeading) + moveY * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
        double frontLeftPower = (rotY + rotX + rotate) / denominator;
        double backLeftPower = (rotY - rotX + rotate) / denominator;
        double frontRightPower = (rotY - rotX - rotate) / denominator;
        double backRightPower = (rotY + rotX - rotate) / denominator;

        //Applies power
        motorFL.setPower(frontLeftPower);
        motorFR.setPower(backLeftPower);
        motorBL.setPower(frontRightPower);
        motorBR.setPower(backRightPower);
    }

    /**
     * Moves the robot based on the orientation of the ROBOT
     * Left joystick moves the robot
     * Right joystick rotates the robot
     * Method also uninverts vertical joystick controls (Up moves up)
     *
     * REMEMBER GP1 CONTROLS MOVEMENT AND ROTATION
     */
    private void robotCentricMovement()
    {
        double moveY = -gp1.left_stick_y;
        double moveX = gp1.left_stick_x;
        double rotate = gp1.right_stick_x;

        //Calculates necessary power
        double denominator = Math.max(Math.abs(moveY) + Math.abs(moveX) + Math.abs(rotate), 1);
        double frontLeftPower = (moveY + moveX + rotate) / denominator;
        double backLeftPower = (moveY - moveX + rotate) / denominator;
        double frontRightPower = (moveY - moveX - rotate) / denominator;
        double backRightPower = (moveY + moveX - rotate) / denominator;

        //Applies Power
        motorFL.setPower(frontLeftPower);
        motorFR.setPower(backLeftPower);
        motorBL.setPower(frontRightPower);
        motorBR.setPower(backRightPower);
    }

}
