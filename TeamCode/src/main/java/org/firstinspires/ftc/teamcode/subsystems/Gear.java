package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.CameraSetup;

public class Gear {
    private CameraSetup camera;
     Servo gearServo;
     Motor intakeMotor;
     Motor transferMotor;
     Motor outtakeMotor;
     Servo angleServo;
    private double servoPos = 0.5;
    private boolean isTrack = false;

    private int targetTagID;

    private double dist;

    private double inPower;
    private double tranPower;
    private double outPower;



    private double trackGain = 0.003;
    private double minPos = 0.25;
    private double maxPos = 0.75;



    private int switchDir = 1;

    private Timer launchTimer;


    public Gear(HardwareMap hw, CameraSetup cam, Motor input, Motor tran, Motor output) {
        gearServo = hw.get(Servo.class, "camServo");
        camera = cam;
        gearServo.setPosition(servoPos);
        intakeMotor = input;
        transferMotor = tran;
        outtakeMotor = output;

        inPower = 0;
        tranPower = 0;
        outPower = 0;

        launchTimer = new Timer();
    }

    public void startTrack(){
        isTrack = true;
    }
    public void finishTrack(){
        isTrack = false;
    }

    public void setTarget(int tagID){
        targetTagID = tagID;
    }

    public void update(){
        if(!isTrack){
            return;
        }
        AprilTagDetection tag = camera.getGoalTag();
        if(tag == null){
            servoPos += switchDir * 0.002;
            if(servoPos < minPos){
                servoPos = minPos;
            }

            if(servoPos > maxPos){
                servoPos = maxPos;
            }
            gearServo.setPosition(servoPos);
            return;
        }
        double yaw = tag.ftcPose.yaw;
        servoPos -= yaw * trackGain;

        if(servoPos < minPos){
            servoPos = minPos;
        }

        if(servoPos > maxPos){
            servoPos = maxPos;
        }

        gearServo.setPosition(servoPos);
        if(Math.abs(yaw) < 1.5){
            lock(tag);

        }


    }

    public void lock(AprilTagDetection tag){
        dist = tag.ftcPose.range;
        if(dist < 10){
            outPower = (15 * Math.PI) * ((19.6 * Math.pow(dist, 2))/((Math.sqrt(3) * dist) - 15.75));
            servoPos = 0.6;
        }
        else{
            angleChange();
        }
    }

    public void angleChange(){
        double velSquared = Math.pow(20, 2); //replace 20
        double distSquared = Math.pow(dist, 2);

        double root = 15.75 + ((9.8 * distSquared) / (2 * velSquared));
        root *= ((2 * 9.8 * distSquared) / velSquared);
        root = Math.sqrt(distSquared - root);

        double angle = velSquared * (dist + root);
        angle /= (9.8 * distSquared);
        angle = Math.atan(angle);

        servoPos = angle;
        outPower = 1;

    }









    public void setAlliance(boolean isRed){ //in init();
        if(isRed){
            switchDir = 1;
        }
        else{
            switchDir = -1;
        }
    }




}















