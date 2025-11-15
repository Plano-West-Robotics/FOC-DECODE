package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;

/**
 * Launcher controls the ball-launching mechanisms for the robot
 * Mechanisms Include:
 * Flywheels that launch the balls
 * Flaps that move the balls into the flywheels
 *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *
 * Should work in both Auto and Tele OpModes
 */
public class Launcher {

    //CLASS CONSTANTS
    public static final double MAX_FLYWHEEL_POWER = 1.0/5.0;
    public static final double FLAP_CLOSED_POS = 1;
    public static final double FLAP_OPEN_POS = 0;

    //HARDWARE
    Servo servoFlap;
    Motor motorFWs;

    //CONTROL VARIABLES
    boolean flywheelsOn;
    boolean isFiring;

    public Launcher(Servo servo, Motor flywheels)
    {
        this.servoFlap = servo;
        this.motorFWs = flywheels;
    }

    /**
     * toggles the flap servo to either scoop the ball or reopen the ramp
     * also updates boolean variable controlling flywheels
     */
    public void changeFlapState()
    {
        if (servoFlap.getPosition() == FLAP_CLOSED_POS)
            servoFlap.setPosition(FLAP_OPEN_POS);
        else
            servoFlap.setPosition(FLAP_CLOSED_POS);
    }

    /**
     * toggles whether or not the flywheels are on
     * also updates boolean variable controlling flywheels
     */
    public void changeFlywheelState()
    {
        flywheelsOn = !flywheelsOn;
        double power;

        if (flywheelsOn)
            power = MAX_FLYWHEEL_POWER;
        else
            power = 0;

        motorFWs.setPower(power);
    }

    /**
     * Automatically fires a ball out of the launcher
     * PREREQS:
     * FLYWHEEL MOTOR IS OFF
     * FLAP IS IN THE CLOSED POSITION
     */
    public void fire()
    {
        changeFlywheelState();
        changeFlapState();

        while (servoFlap.getPosition() != FLAP_OPEN_POS)
        {

        }

        changeFlapState();
        changeFlywheelState();
    }

    /**
     * Checks if the robot is firing
     * @return
     */
    public boolean isFiring()
    {
        return motorFWs.getPower() != 0;
    }

}
