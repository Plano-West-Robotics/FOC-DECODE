package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Motor;

public class LauncherTwo {

    //CLASS CONSTANTS (not yet finalized)
    public static double inoutPower = 0.6;
    public static double transferPower = 0.2;

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
         double power;

         if(inoutOn){
             power = inoutPower;
         }
         else{
             power = 0;
         }
         intakeMotor.setPower(power);
         outtakeMotor.setPower(power);
     }


    /**
     * Toggles whether the transfer is on or not
     * Updates the boolean controlling the transfer
     */
    public void transferChange(){
         transferOn = !transferOn;
         double power;

         if(transferOn){
             power = transferPower;
         }
         else{
             power = 0;
         }
         transferMotor.setPower(power);
     }

}
