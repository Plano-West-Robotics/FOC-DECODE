package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.CameraSetup;

public class Gear {

    //CONSTANTS
    private static final double GRAV = 9.8;
    private static final double GOAL_HEIGHT = 15;
    private static final double ROBOT_HEIGHT = 0;
    private static final double MAX_VELOCITY = 20;
    private static final double MAX_ANGLE = 60;
    private static final double TRUE_MAX_ANGLE = 180;
    private static final double MAX_MOTOR_RATIO = 1;
    private static final double RADIUS = 2;


    //VARIABLES
    private CameraSetup camera;
     Servo gearServo;
     Motor intakeMotor;
     Motor transferMotor;
     Motor outtakeMotor;
     Servo angleServo;
    private double servoPos;
    private boolean isTrack = false;

    private int targetTagID;

    private double dist;

    private double inPower;
    private double tranPower;
    private double outPower;



    private double trackGain = 0.003;
    private double minPos = 0.25;
    private double maxPos = 0.75;

    private double angleServoPos;


    private int switchDir;

    private Timer launchTimer;


    public Gear(HardwareMap hw, CameraSetup cam, Motor input, Motor tran, Motor output) {
        gearServo = hw.get(Servo.class, "camServo");
        angleServo = hw.get(Servo.class, "angleServo");
        camera = cam;
        gearServo.setPosition(servoPos);
        intakeMotor = input;
        transferMotor = tran;
        outtakeMotor = output;

        inPower = 0;
        tranPower = 0;
        outPower = 0;

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

    public void lock(AprilTagDetection tag){
        dist = tag.ftcPose.range;
        if(dist > -1) //SET TO -1 SO THAT IT ONLY EVER CHANGES VELOCITY
        {
            outPower = (MAX_ANGLE / ( 2 * Math.PI * RADIUS)) * ((2 * GRAV * Math.pow(dist, 2))/((Math.tan(MAX_ANGLE) * dist) - (GOAL_HEIGHT-ROBOT_HEIGHT)));
            angleServoPos = MAX_ANGLE / TRUE_MAX_ANGLE;
        }
        else{
            angleChange();
        }
    }

    public void velocityChange()
    {
        outPower = (MAX_ANGLE / ( 2 * Math.PI * RADIUS));
        outPower *= (2 * GRAV * Math.pow(dist, 2));
        outPower /= (Math.tan(MAX_ANGLE) * dist) - (GOAL_HEIGHT-ROBOT_HEIGHT);
        angleServoPos = MAX_ANGLE / TRUE_MAX_ANGLE;
    }
    public void angleChange(){
        double velSquared = Math.pow(MAX_VELOCITY, 2); //replace 20
        double distSquared = Math.pow(dist, 2);

        double root = (GOAL_HEIGHT-ROBOT_HEIGHT) + ((GRAV * distSquared) / (2 * velSquared));
        root *= ((2 * GRAV * distSquared) / velSquared);
        root = Math.sqrt(distSquared - root);

        double angle = velSquared * (dist + root);
        angle /= (GRAV * distSquared);
        angle = Math.atan(angle);

        angleServoPos = angle;
        outPower = MAX_MOTOR_RATIO;

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















