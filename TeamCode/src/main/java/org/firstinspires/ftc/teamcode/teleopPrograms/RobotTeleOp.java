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

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "TeleOp v1.0.0.0", group = "Main")
public class RobotTeleOp extends OpMode
{
    //Enums
    //Movement Direciton Enum
    private enum Direction
    {
        //DEFAULT STATE
        DEFAULT,

        //CARDINAL MOVEMENT (NOT CURRENTLY NEEDED)
        FORWARD,
        LEFT,
        BACKWARD,
        RIGHT,

        //DIAGONAL MOVEMENT
        FOR_LEFT,
        FOR_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,

        //ROTATIONS
        ROT_LEFT,
        ROT_RIGHT
    }

    //Drive Mode Enum
    private enum DriveMode
    {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    //Declaring Misc Hardware
    IMU imu;

    //Declaring Motors
    Motor motorFL;
    Motor motorFR;
    Motor motorBL;
    Motor motorBR;

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

    //Declaring State controllers
    DriveMode movementMode;
    Direction direction;



    @Override
    public void init() {
        //Declaring hardware parts + gamepads
        this.motorFL = new Motor(hardwareMap,"motorFL");
        this.motorFR = new Motor(hardwareMap,"motorFR");
        this.motorBL = new Motor(hardwareMap,"motorBL");
        this.motorBR = new Motor(hardwareMap,"motorBR");

        //Servos aren't currently on the robot and thus are commented out.

        /*
        this.servo1 = hardwareMap.get(Servo.class, "servo1");
        this.servo2 = hardwareMap.get(Servo.class, "servo2");
        */

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);


        gp1 = new Gamepad();
        gp2 = new Gamepad();
        prevGp1 = new Gamepad();
        prevGp2 = new Gamepad();

        //Declaring starting enum states
        movementMode = DriveMode.FIELD_CENTRIC;
        direction = Direction.DEFAULT;

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

        //State Machine Handler (Currently Unused)
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
        double denominator = Math.max(Math.abs(moveY) + Math.abs(moveX) + Math.abs(rotate), 1);
        double frontLeftPower = (moveY + moveX + rotate) / denominator;
        double backLeftPower = (moveY - moveX + rotate) / denominator;
        double frontRightPower = (moveY - moveX - rotate) / denominator;
        double backRightPower = (moveY + moveX - rotate) / denominator;

        //Compensates for orientation of robot relative to the field


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
