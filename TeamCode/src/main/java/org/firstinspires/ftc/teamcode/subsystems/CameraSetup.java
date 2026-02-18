package org.firstinspires.ftc.teamcode.subsystems;


import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraSetup {

    private VisionPortal visPort;
    private AprilTagProcessor atp;

    public static final double FX = 8.245077776187888503e2;
    public static final double FY = 8.270054620332543891e2;
    public static final double CX = 3.212181651421647643e2;
    public static final double CY = 2.425865530229235674e2;

    public static final int TAG_BLUE = 20;
    public static int TAG_21 = 21;
    public static int TAG_22 = 22;
    public static int TAG_23 = 23;
    public static final int TAG_RED = 24;

    public int tagID;

    private double xOff = 0;
    private double yOff = 0;
    private double dist = 0;

    private double yaw; //THIS IS FOR LOCKING WHEN SCORING

    private boolean hasTag = false;


    private int lostFrames = 0;
    private int maxLost = 5;



    public CameraSetup(HardwareMap hardwaremap){

        atp = new AprilTagProcessor.Builder().setLensIntrinsics(FX, FY, CX, CY).setDrawAxes(false).setDrawCubeProjection(false).setDrawTagOutline(false).build();

        visPort = new VisionPortal.Builder().setCamera(hardwaremap.get(WebcamName.class, "Webcam")).setCameraResolution(new Size(640, 480)).addProcessor(atp).build();

    }

    public void update(){
        List<AprilTagDetection> detections = atp.getDetections();

        boolean alreadyFrame = false;

        for (AprilTagDetection tag: detections){
            if (tag.ftcPose == null){
                continue;
            }
            if(tag.id == TAG_BLUE || tag.id == TAG_RED || tag.id == TAG_21 || tag.id == TAG_22 ||tag.id == TAG_23 ){
                xOff = tag.ftcPose.x;
                yOff = tag.ftcPose.y;
                dist = tag.ftcPose.range;
                yaw = tag.ftcPose.yaw;
                tagID = tag.id;

                hasTag = true;
                lostFrames = 0;
                alreadyFrame = true;
                break;
            }
        }
        if(!alreadyFrame){
            lostFrames++;
            if (lostFrames > maxLost){
                hasTag = false;
            }
        }


    }

    public boolean hasTagCheck(){
        return hasTag;
    }

    public double getXOff(){
        return xOff;
    }

    public double getYOff(){
        return yOff;
    }

    public double getDist(){
        return dist;
    }

    public int getTag(){
        return tagID;
    }

    public boolean hasGoalTagSet(){
        return tagID == TAG_BLUE || tagID == TAG_RED;
    }

    public double getYaw(){
        return yaw;
    }

    public void stop(){
        visPort.close();
    }



}
