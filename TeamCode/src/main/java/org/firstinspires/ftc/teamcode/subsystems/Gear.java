package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.CameraSetup;

public class Gear {

    //CONSTANTS

    private static final double MAX_VELOCITY = 20;
    private static final double MAX_MOTOR_RATIO = 1;
    private static final double RADIUS = 2;

    private static final double trackGain = 0.003;
    private static final double minPos = 0.25;
    private static final double maxPos = 0.75;


    //VARIABLES
    private CameraSetup camera;
    private Servo gearServo;

    private double servoPos;
    private boolean isTrack = false;

    private int targetTagID;

    private double dist;

    private double inPower;
    private double tranPower;
    private double outPower;

    private double angleServoPos;


    private int switchDir;

    private Timer launchTimer;


    public Gear(Servo gear, CameraSetup cam) {
        gearServo = gear;
        camera = cam;
        gearServo.setPosition(servoPos);

        switchDir = 1;
        servoPos = 0.5;

        launchTimer = new Timer();
    }

    public void startTrack(){
        isTrack = true;
    }
    public void finishTrack(){
        isTrack = false;
    }

    public void setTarget(int tagID) {
        targetTagID = tagID;
    }

    public void update(){
        if(!isTrack){
            return;
        }

        AprilTagDetection tag = camera.getGoalTag();

        if(tag == null){
            servoPos += switchDir * 0.002; //MOVE GEAR BY 1 TICK.
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








    public void setAlliance(boolean isRed){ //in init();
        if(isRed){
            setTarget(24);
            switchDir = 1;
        }
        else{
            setTarget(20);
            switchDir = -1;
        }
    }




}















