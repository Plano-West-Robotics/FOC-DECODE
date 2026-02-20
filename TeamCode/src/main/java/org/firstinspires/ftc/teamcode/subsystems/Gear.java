package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.CameraSetup;

public class Gear {
    private CameraSetup camera;
    private Servo gearServo;
    private double servoPos = 0.5;
    private boolean isTrack = false;

    private int targetTagID;

    private double xOff;
    private double yOff;
    private double dist;

    private double trackGain = 0.003;
    private double minPos = 0.25;
    private double maxPos = 0.75;

    private int switchDir = 1;


    public Gear(HardwareMap hw, CameraSetup cam) {
        gearServo = hw.get(Servo.class, "camServo");
        camera = cam;
        gearServo.setPosition(servoPos);
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
            //Comment
        }


    }

    public void lock(AprilTagDetection tag){
        dist = tag.ftcPose.range;
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















