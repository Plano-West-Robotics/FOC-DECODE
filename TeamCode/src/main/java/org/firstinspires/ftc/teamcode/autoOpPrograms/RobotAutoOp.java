package org.firstinspires.ftc.teamcode.examplePedroAuto;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.NINETY;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.ONEEIGHTY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@Autonomous(name = "Robot Autonomous", group = "Main")
public class RobotAutoOp extends OpMode {

    //Hardware Instance Variables
    Motor motorFWs;

    Servo servoFlap;

    //Subsystems Instance Variables
    Launcher launcher;

    //Pedro Pathing Instance Variables
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private BasicStates pathState;

    private Pose startPose = new Pose(56, 8);
    private Pose toArtifact1 = new Pose(56, 8);
    private Pose toArtifact2 = new Pose(61.4, 83.627);
    private Pose toArtifact3 = new Pose(29.292, 83.627);

    private Pose toLaunch1 = new Pose(29.292, 83.627);
    private Pose toLaunch2 = new Pose(43.602, 98.832);

    private Pose toEnd1 = new Pose(43.602, 98.832);
    private Pose toEnd2 = new Pose(81.391, 58.807);
    private Pose toEnd3 = new Pose(39.000, 34.000);

    private Path toArtifactPath;
    private Path toLaunchPath;
    private Path toEndPath;

    public static final int INIT_SECONDS = 1;
    public static final int LAUNCH_SECONDS = 3;

    public void buildPaths()
    {

        toArtifactPath = new Path(
                new BezierCurve(
                        toArtifact1,
                        toArtifact2,
                        toArtifact3
                )
        );
        toArtifactPath.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));

        toLaunchPath = new Path(
                new BezierLine(
                        toLaunch1,
                        toLaunch2
                )
        );
        toLaunchPath.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135));

        toEndPath = new Path(
                new BezierCurve(
                        toEnd1,
                        toEnd2,
                        toEnd3
                )
        );
        toEndPath.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180));
    }

    public void autoFSM() {
        switch(pathState)
        {
            case INIT:
                if (opmodeTimer.getElapsedTime() >= INIT_SECONDS)
                    setPathState(BasicStates.TO_ARTIFACT);
                break;
            case TO_ARTIFACT:
                follower.followPath(toArtifactPath, true); // holdEnd means it will stay at the point after reaching it

                // move to the next state when the path has completed
                if (!follower.isBusy())
                    setPathState(BasicStates.TO_SCORING);
                break;
            case TO_SCORING:
                follower.followPath(toLaunchPath, true);

                if (!follower.isBusy())
                    setPathState(BasicStates.AT_SCORING);
                break;
            case AT_SCORING:
                if(pathTimer.getElapsedTime() >= LAUNCH_SECONDS)
                    setPathState(BasicStates.PARK);
                break;
            case PARK:
                follower.followPath(toEndPath, true);

                if (!follower.isBusy())
                    setPathState(BasicStates.IDLE);
                break;
            case IDLE:
                break;
            default:
                System.out.println("FSM System reached an undefined state");
                break;
        }
    }

    public void setPathState(BasicStates state)
    {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        //Timers
        this.pathTimer = new Timer();
        this.opmodeTimer = new Timer();
        this.opmodeTimer.resetTimer();

        //Hardware
        this.motorFWs = new Motor(hardwareMap, "motorFWs");
        this.servoFlap = new Servo(hardwareMap, "servoFlap");

        //Subsystems
        this.launcher = new Launcher(servoFlap, motorFWs);

        this.follower = Constants.createFollower(hardwareMap);
        buildPaths();
        this.follower.setStartingPose(startPose);
    }

    @Override
    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(BasicStates.INIT);
    }

    @Override
    public void loop() {
        follower.update();
        autoFSM(); // need to constantly check the state machine status

        // as the program is running, you can view the status of the robot for debugging
        telemetry.addData("path state:", pathState);
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("heading:", follower.getPose().getHeading());
        telemetry.update();
    }





}
