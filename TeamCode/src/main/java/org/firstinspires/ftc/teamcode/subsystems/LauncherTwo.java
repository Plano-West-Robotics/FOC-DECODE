package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Motor;

public class LauncherTwo {

    //CLASS CONSTANTS (not yet finalized)
    public static double inPower = 1;
    public static double outPower = 0.75;
    public static double transferPower = 1;

    //HARDWARE
     Motor intakeMotor;
     Motor transferMotor;
     Motor outtakeMotor;

     //CONTROL VARIABLES
    boolean inoutOn;
    boolean transferOn;

     public LauncherTwo(Motor input, Motor transfer, Motor output){
         this.intakeMotor = input;
         this.transferMotor = transfer;
         this.outtakeMotor = output;

         this.transferMotor.reverse();

         intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
         transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
         outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
     }

    /**
     * Toggles whether the intake/outtake is on or not
     * Updates the boolean controlling the intake/outtake
     */
    public void inOutChange(){
         inoutOn = !inoutOn;
         double powIn;
         double powOut;

         if(inoutOn){
             powIn = inPower;
             powOut = outPower;
         }
         else{
             powIn = 0;
             powOut = 0;
         }
         intakeMotor.setPower(powIn);
         outtakeMotor.setPower(powOut);
     }


    /**
     * Toggles whether the transfer is on or not
     * Updates the boolean controlling the transfer
     */
    public void transferChange(){
         transferOn = !transferOn;
         double power;

         if(transferOn){
             power = -transferPower;
         }
         else{
             power = 0;
         }
         transferMotor.setPower(power);
     }

}
