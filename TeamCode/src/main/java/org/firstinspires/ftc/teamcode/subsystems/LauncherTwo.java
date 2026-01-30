package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Motor;

public class LauncherTwo {

    //CLASS CONSTANTS (not yet finalized)
    public static double inPower = -0.3;
    public static double outPower = 0.65;
    public static double transferPower = 1;

    //HARDWARE
     Motor intakeMotor;
     Motor transferMotor;
     Motor outtakeMotor;

     //CONTROL VARIABLES
    boolean inOn;

    boolean outOn;
    boolean transferOn;

     public LauncherTwo(Motor input, Motor transfer, Motor output){
         this.intakeMotor = input;
         this.transferMotor = transfer;
         this.outtakeMotor = output;

         this.transferMotor.reverse();

         intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
         outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
     }

    /**
     * Toggles whether the intake/outtake is on or not
     * Updates the boolean controlling the intake/outtake
     */
    public void inChange(){
         inOn = !inOn;
         intakeMotor.setPower(inOn ? inPower : 0);
     }

    public void outChange(){
        outOn = !outOn;
        double powOut;

        outtakeMotor.setPower(outOn ? outPower : 0);
    }


    /**
     * Toggles whether the transfer is on or not
     * Updates the boolean controlling the transfer
     */
    public void transferChange(){
         transferOn = !transferOn;
         double power;

         transferMotor.setPower(transferOn ? -transferPower : 0);
         if(transferOn){
             power = -transferPower;
         }
         else{
             power = 0;
         }
         transferMotor.setPower(power);
     }

}
