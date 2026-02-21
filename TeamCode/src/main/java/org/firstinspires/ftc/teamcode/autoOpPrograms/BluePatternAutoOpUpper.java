package org.firstinspires.ftc.teamcode.autoOpPrograms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.CameraSetup;
import org.firstinspires.ftc.teamcode.subsystems.Gear;
import org.firstinspires.ftc.teamcode.subsystems.LauncherTwo;

@Autonomous(name = "Red AutoOp with Top Start", group = "Autonomous")
public class BluePatternAutoOpUpper extends OpMode {

    //State Enum
    public enum BasicStates
    {
        INIT,
        TO_MOTIF,
        SCANNING_MOTIF,
        TO_ARTIFACT,
        COLLECTING,
        TO_SCORING,
        FIRING,
        PARK,
        IDLE
    }

    //Pose & Angle Constants
    private final double TOP_START_ANGLE = Math.toRadians(144);
    private final double BOT_START_ANGLE = Math.toRadians(90);
    private final double COLLECTION_ANGLE = Math.toRadians(0);
    private final double TOP_LAUNCH_ANGLE = Math.toRadians(134);
    private final double BOT_LAUNCH_ANGLE = Math.toRadians(112);
    private final double SCAN_ANGLE = Math.toRadians(90);
    private final double END_ANGLE = Math.toRadians(90);

    private final Pose TOP_START_POSE = new Pose(21, 123);
    private final Pose BOT_START_POSE = new Pose(56, 8);
    private final Pose TOP_ARTI_POSE = new Pose(44,84);
    private final Pose MID_ARTI_POSE = new Pose(44,60);
    private final Pose BOT_ARTI_POSE = new Pose(44,36);
    private final Pose TOP_SCORING_POSE = new Pose(59,85);
    private final Pose BOT_SCORING_POSE = new Pose(62,12);
    private final Pose SCANNING_POSE = new Pose(59,85);
    private final Pose END_POSE = new Pose(44,30);
    private final int COLLECTION_DISTANCE = 32;

    //Timer Constants

    public static final int INIT_SECONDS = 1;
    public static final int LAUNCH_SECONDS = 5;
    public static final int ROUND_SECONDS = 27;


    //Hardware Instance Variables
    Motor motorIN;
    Motor motorTFER;
    Motor motorOUT;

    //Subsystems Instance Variables
    LauncherTwo launcher;

    //Pedro Pathing Instance Variables
    private Follower follower;
    private CameraSetup camera;

    private Gear gear;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public boolean preloaded;
    private BasicStates pathState;
    private int motifID;

    private Path collectBallsPath;
    private Path toArtifactPath;
    private Path toScanPath;
    private Path toScorePath;
    private Path toEndPath;

    /**
     * CURRENTLY UNUSED
     */
    public void buildPaths()
    {
        toScorePath = new Path(
                new BezierLine(
                        SCANNING_POSE,
                        BOT_SCORING_POSE
                )
        );
        toScorePath.setLinearHeadingInterpolation(SCAN_ANGLE, BOT_LAUNCH_ANGLE);

        toScanPath = new Path(
                new BezierLine(
                        TOP_START_POSE,
                        SCANNING_POSE
                )
        );
        toScanPath.setLinearHeadingInterpolation(TOP_START_ANGLE, SCAN_ANGLE);

    }

    public void autoFSM() {
        if (opmodeTimer.getElapsedTimeSeconds() >= ROUND_SECONDS && pathState != BasicStates.PARK && pathState != BasicStates.IDLE)
        {
            motorIN.setPower(0);
            motorTFER.setPower(0);
            motorOUT.setPower(0);
            generateEndPath();
            setPathState(BasicStates.PARK);
        }

        switch (pathState) {
            case INIT:
                if (opmodeTimer.getElapsedTime() >= INIT_SECONDS)
                    setPathState(BasicStates.TO_MOTIF);
                break;

            case TO_MOTIF:
                follower.followPath(toScanPath, true);

                if(!follower.isBusy())
                    setPathState(BasicStates.SCANNING_MOTIF);
                break;

            case SCANNING_MOTIF:
                if(pathTimer.getElapsedTime() < 0.4){
                    break;
                }
                //Do stuff with camera to set motifID to the correct id.
                if (camera.hasTagCheck()){
                    motifID = camera.getTag();
                }

                boolean scanned = false;
                //Set toArtifactPath to the proper artifacts
                switch(motifID)
                {   //MOTIFS MATCH...
                    case 21: //TOP ARTIFACT PATTERN
                        toArtifactPath = new Path(
                                new BezierLine(
                                        BOT_SCORING_POSE,
                                        TOP_ARTI_POSE
                                )
                        );
                        toArtifactPath.setLinearHeadingInterpolation(BOT_LAUNCH_ANGLE, COLLECTION_ANGLE);

                        collectBallsPath = new Path(
                                new BezierLine(
                                        TOP_ARTI_POSE,
                                        new Pose(TOP_ARTI_POSE.getX() - COLLECTION_DISTANCE, TOP_ARTI_POSE.getY())
                                )
                        );
                        collectBallsPath.setLinearHeadingInterpolation(COLLECTION_ANGLE, COLLECTION_ANGLE);
                        scanned = true;
                        break;

                    case 22: //MIDDLE ARTIFACT PATTERN
                        toArtifactPath = new Path(
                                new BezierLine(
                                        BOT_SCORING_POSE,
                                        MID_ARTI_POSE
                                )
                        );
                        toArtifactPath.setLinearHeadingInterpolation(BOT_LAUNCH_ANGLE,COLLECTION_ANGLE);

                        collectBallsPath = new Path(
                                new BezierLine(
                                        MID_ARTI_POSE,
                                        new Pose(MID_ARTI_POSE.getX() - COLLECTION_DISTANCE, MID_ARTI_POSE.getY())
                                )
                        );
                        collectBallsPath.setLinearHeadingInterpolation(COLLECTION_ANGLE, COLLECTION_ANGLE);
                        scanned = true;
                        break;

                    case 23: //BOTTOM ARTIFACT PATTERN
                        toArtifactPath = new Path(
                                new BezierLine(
                                        BOT_SCORING_POSE,
                                        BOT_ARTI_POSE
                                        )
                        );
                        toArtifactPath.setLinearHeadingInterpolation(BOT_LAUNCH_ANGLE,COLLECTION_ANGLE);

                        collectBallsPath = new Path(
                                new BezierLine(
                                        BOT_ARTI_POSE,
                                        new Pose(BOT_ARTI_POSE.getX() - COLLECTION_DISTANCE, BOT_ARTI_POSE.getY())
                                )
                        );
                        collectBallsPath.setLinearHeadingInterpolation(COLLECTION_ANGLE, COLLECTION_ANGLE);
                        scanned = true;
                        break;

                    default: //Defaults to collecting the bottom artifacts currently
                        toArtifactPath = new Path(
                                new BezierLine(
                                        follower.getPose(),
                                        BOT_ARTI_POSE
                                )
                        );
                        toArtifactPath.setLinearHeadingInterpolation(follower.getHeading(), COLLECTION_ANGLE);

                        collectBallsPath = new Path(
                                new BezierLine(
                                        BOT_ARTI_POSE,
                                        new Pose(BOT_ARTI_POSE.getX() + COLLECTION_DISTANCE, BOT_ARTI_POSE.getY())
                                )
                        );
                        collectBallsPath.setLinearHeadingInterpolation(COLLECTION_ANGLE, COLLECTION_ANGLE);
                        scanned = true;
                        break;
                }

                //Dont forget to wait until it finishes scanning
                if (scanned)
                {
                    setPathState(BasicStates.TO_SCORING);
                }

                break;


            case TO_ARTIFACT:
                follower.followPath(toArtifactPath, true);
                // holdEnd means it will stay at the point after reaching it

                // move to the next state when the path has completed
                if (!follower.isBusy())
                    setPathState(BasicStates.COLLECTING);
                break;

            case COLLECTING:
                motorIN.setPower(-0.1);
                motorTFER.setPower(0.26);
                follower.followPath(collectBallsPath, true);

                if (!follower.isBusy())
                    setPathState(BasicStates.TO_SCORING);
                break;

            case TO_SCORING:
                motorIN.setPower(0);
                motorOUT.setPower(0);
                follower.followPath(toScorePath, true);

                if (!follower.isBusy())
                    setPathState(BasicStates.FIRING);
                break;


            case FIRING:
                if(!camera.hasGoalTagSet()){
                    actionTimer.resetTimer();
                    motorIN.setPower(0);
                    motorTFER.setPower(0);
                    motorOUT.setPower(0);
                    break;
                }
                double robDis = camera.getDist();
                if(robDis < 10 || robDis > 80){
                    motorIN.setPower(0);
                    motorTFER.setPower(0);
                    motorOUT.setPower(0);
                    break;
                }
                double outPower = (15 * Math.PI) * ((19.6 * Math.pow(robDis, 2))/((Math.sqrt(3) * robDis) - 15.75));
                outPower = Math.max(0.0, Math.min(outPower, 1));
                motorIN.setPower(-0.1);
                motorTFER.setPower(0.26);
                motorOUT.setPower(outPower);

                if (actionTimer.getElapsedTime() >= LAUNCH_SECONDS)
                {

                    motorIN.setPower(0);
                    motorTFER.setPower(0);
                    motorOUT.setPower(0);

                    if (preloaded) {
                        setPathState(BasicStates.TO_ARTIFACT);
                        preloaded = false;
                    }
                    else
                    {
                        generateEndPath();
                        setPathState(BasicStates.PARK);
                    }
                }

                break;

            case PARK:
                follower.followPath(toEndPath, true);

                if (!follower.isBusy())
                    setPathState(BasicStates.IDLE);
                break;


            case IDLE:
                //Used to wait for the end of AutoOp
                break;


            default:
                System.out.println("FSM System reached an undefined state");
                motorIN.setPower(0);
                motorTFER.setPower(0);
                motorOUT.setPower(0);
                follower.followPath(toEndPath, true);
                break;
        }
    }

    public void setPathState(BasicStates state)
    {
        pathState = state;
        pathTimer.resetTimer();

        if (state == BasicStates.FIRING){
            actionTimer.resetTimer();
        }
    }

    @Override
    public void init() {
        //Timers
        this.pathTimer = new Timer();
        this.opmodeTimer = new Timer();
        this.actionTimer = new Timer();
        this.opmodeTimer.resetTimer();

        //Hardware
        this.motorOUT = new Motor(hardwareMap, "o");
        this.motorIN = new Motor(hardwareMap, "i");
        this.motorTFER = new Motor(hardwareMap,"t");

        //Subsystems
        this.launcher = new LauncherTwo(motorIN, motorTFER, motorOUT);

        this.follower = Constants.createFollower(hardwareMap);
        this.follower.setStartingPose(TOP_START_POSE);
        buildPaths();
        this.camera = new CameraSetup(hardwareMap);
        this.gear = new Gear(hardwareMap,camera, motorIN, motorTFER, motorOUT);
        camera.setAlliance(false);
        gear.setAlliance(false);
    }

    @Override
    public void start()
    {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        pathTimer.resetTimer();
        setPathState(BasicStates.INIT);

        preloaded = true;
    }

    @Override
    public void loop() {
        follower.update();
        camera.update();
        autoFSM(); // need to constantly check the state machine status

        // as the program is running, you can view the status of the robot for debugging
        telemetry.addData("path state:", pathState);
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("heading:", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * Generates a path from the robot's current position to its final pose
     */
    private void generateEndPath()
    {
        toEndPath = new Path(
                new BezierLine(
                        follower.getPose(),
                        END_POSE
                )
        );
        toEndPath.setLinearHeadingInterpolation(follower.getHeading(), END_ANGLE);
    }
}
