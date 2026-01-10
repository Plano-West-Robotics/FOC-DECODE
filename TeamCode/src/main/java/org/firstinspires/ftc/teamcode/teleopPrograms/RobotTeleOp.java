package org.firstinspires.ftc.teamcode.teleopPrograms;

/*
 * Working TeleOp Control program for the FOC Robot
 * Has working Field-Centric and Robot-Centric Control mode
 * Also has controls for a vertical flywheel setup and a servo to push balls into the flywheels
 *
 * @version 1.1.2.5
 * @date 12/1/2025
 */

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.LauncherTwo;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name = "TeleOp v1.1.2.5", group = "TeleOp")
public class RobotTeleOp extends OpMode
{
    //CONSTANTS

    //ENUMS

    //Used to determine which movement mode the robot is in
    private enum DriveMode
    {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    //INSTANCE VARIABLES

    //Declaring Misc Hardware
    IMU imu; //Orientation is finalized for meet

    //Declaring Motors
    //Movement Motors
    Motor motorFL; // CH Port 3
    Motor motorFR; // CH Port 2
    Motor motorBL; // CH Port 1
    Motor motorBR; // CH Port 0

    //Intake / Outtake Motors Motors
    Motor motorOUT; // EH Port 3
    Motor motorIN; //EH Port 2

    Motor motorTFER; //EH Port 1

    //Declaring Servos
     CRServo servoLoader; // EH Servo Port 5??

    //Declaring Subsystems
    Launcher launcher;
    LauncherTwo launcher2;

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
    boolean inoutOn;
    boolean transferOn;

    @Override
    public void init() {
        //Declaring hardware parts + gamepads

        //Movement motors
        this.motorFL = new Motor(hardwareMap,"fl");
        this.motorFR = new Motor(hardwareMap,"fr");
        this.motorBL = new Motor(hardwareMap,"bl");
        this.motorBR = new Motor(hardwareMap,"br");

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

        //Code for intake and outtake;
        this.motorOUT = new Motor(hardwareMap, "o");
        this.motorIN = new Motor(hardwareMap, "i");
        this.motorTFER = new Motor(hardwareMap,"t"); //Just added this in case

        //Reversing left motors.
        this.motorFL.reverse();
        this.motorBL.reverse();

        //CRServos
        this.servoLoader = hardwareMap.get(CRServo.class, "servoLoader");

        this.imu = hardwareMap.get(IMU.class, "imu");

        //Make sure to change LogoFacingDirection and UsbFacingDirection once robot is built
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        this.imu.initialize(parameters);
        this.imu.resetYaw();

        //Subsystems
        //this.launcher = new Launcher(servoLoader, motorOUT); //Currently no servo because L
        this.launcher2 = new LauncherTwo(motorIN, motorTFER, motorOUT);

        this.gp1 = new Gamepad();
        this.gp2 = new Gamepad();
        this.prevGp1 = new Gamepad();
        this.prevGp2 = new Gamepad();

        //Declaring starting enum states
        this.movementMode = DriveMode.FIELD_CENTRIC;

        this.flywheelsOn = false;
        this.inoutOn = false;
        this.transferOn = false;

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
        /// GAMEPAD 1 CONTROLS
        /// Left Stick - Directional Movement
        /// Right Stick - Rotational Movement
        /// A - Field Centric Drive Mode
        /// B - Robot Centric Drive Mode
        /// X - Toggles Intake/Outtake (Loading/Output Mechanism)
        /// Y - Toggles Transfer State (Transfer Mechanism)
        if (gp1.a && !prevGp1.a)
            movementMode = DriveMode.FIELD_CENTRIC;
        else if (gp1.b && !prevGp1.b)
            movementMode = DriveMode.ROBOT_CENTRIC;
        if (gp1.left_bumper && !prevGp1.left_bumper)
            //launcher.changeFlywheelState();
            launcher2.inChange();
        if (gp1.right_bumper && !prevGp1.right_bumper)
            //launcher.changeFlywheelState();
            launcher2.outChange();

        if (gp1.y && !prevGp1.y)
        {
            launcher2.transferChange();
            /*if (motorIN.getPower() == 1)
                motorIN.setPower(0);
            else
                motorIN.setPower(1);
            launcher.rotateFlap();*/
        }
    //  if (gp1.y && !prevGp1.y)
    //      launcher.changeFlapState();

    }

    /**
     * Moves the robot based on the orientation of the FIELD
     * MOVEMENT CONTROLS
     * Left joystick moves the robot
     * Right joystick rotates the robot
     * Method also uninverts vertical joystick controls (Up moves up)
     * REMINDERS
     * REMEMBER GP1 CONTROLS MOVEMENT AND ROTATION
     * GP2 DOES CANNOT CONTROL ROBOT MOVEMENT
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
        motorBL.setPower(backLeftPower);
        motorFR.setPower(frontRightPower);
        motorBR.setPower(backRightPower);

        //Telemetry
        telemetry.addLine("fL: " + frontLeftPower);
        telemetry.addLine("fR: " + frontRightPower);
        telemetry.addLine("bR: " + backRightPower);
        telemetry.addLine("bL: " + backLeftPower);
    }

    /**
     * Moves the robot based on the orientation of the ROBOT
     * MOVEMENT CONTROLS
     * Left joystick moves the robot
     * Right joystick rotates the robot
     * Method also uninverts vertical joystick controls (Up moves up)
     * REMINDERS
     * REMEMBER GP1 CONTROLS MOVEMENT AND ROTATION
     * GP2 DOES CANNOT CONTROL ROBOT MOVEMENT
     */
    private void robotCentricMovement()
    {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);


       /* double moveY = -gp1.left_stick_y;
        double moveX = gp1.left_stick_x * 1.1;
        double rotate = gp1.right_stick_x; */


    /*
        //Calculates necessary power
        double denominator = Math.max(Math.abs(moveY) + Math.abs(moveX) + Math.abs(rotate), 1);
        double frontLeftPower = (moveY + moveX + rotate) / denominator;
        double backLeftPower = (moveY - moveX + rotate) / denominator;
        double frontRightPower = (moveY - moveX - rotate) / denominator;
        double backRightPower = (moveY + moveX - rotate) / denominator;


        double frontLeftPower = (moveY + rotate);
        double backLeftPower = (moveY + rotate);
        double frontRightPower = (moveY - rotate);
        double backRightPower = (moveY - rotate);

        //Applies Power
        motorFL.setPower(frontLeftPower);
        motorBL.setPower(frontLeftPower);
        motorFR.setPower(frontRightPower);
        motorBR.setPower(frontRightPower);
        */
    }

}