package org.firstinspires.ftc.teamcode.examplePedroAuto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.NINETY;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.ONEEIGHTY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


// note - make sure to make the robot size 18 by 18
// note 2 - pedro's coordinates are like a normal y vs x plot

@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private BasicStates pathState;

    private final Pose startP = new Pose(56, 9); // the bot's starting location on the field
    private final Pose pickup1_1P = new Pose(58, 28); // intermediate curve point
    private final Pose pickup1_2P = new Pose(42, 36); // final curve point (ends by facing artifacts)
    private final Pose pickup1_3P = new Pose(16, 36); // runs over artifacts

    private final Pose launch1_1P = new Pose(64, 20); // intermediate on score curve
    private final Pose launch1_2P = new Pose(65, 80); // final curve point

    private final Pose parkP = new Pose(40, 34); // parking spot

    private Path goToArtifacts;
    private PathChain goToScoring;
    private Path park;

    private boolean resetTimer = true;

    public static final int INIT_SECONDS = 1;
    public static final int LAUNCH_SECONDS = 3;

    public void buildPedroPaths()
    {
        goToArtifacts = new Path(
                new BezierCurve(startP, pickup1_1P, pickup1_2P));

        goToArtifacts.setLinearHeadingInterpolation(NINETY, ONEEIGHTY);


        goToScoring = follower.pathBuilder()
                .addPath(new BezierLine(pickup1_2P, pickup1_3P))
                .setConstantHeadingInterpolation(ONEEIGHTY)
                .addPath(new BezierCurve(pickup1_3P, launch1_1P, launch1_2P))
                .setLinearHeadingInterpolation(ONEEIGHTY, Math.toRadians(135))
                .build();


        park = new Path(
                new BezierLine(launch1_2P, parkP));

        park.setLinearHeadingInterpolation(Math.toRadians(135), NINETY);
    }

    public void autonomousStateMachine()
    {
        switch(pathState)
        {
            case INIT:
                if (pathTimer.getElapsedTimeSeconds() >= INIT_SECONDS) pathState = BasicStates.TO_ARTIFACT;
                break;

            case TO_ARTIFACT:
                follower.followPath(goToArtifacts, true);
                if (!follower.isBusy()) pathState = BasicStates.TO_SCORING;
                break;

            case TO_SCORING:
                follower.followPath(goToScoring, true);

                if (!follower.isBusy() && resetTimer)
                {
                    pathTimer.resetTimer();
                    resetTimer = false;
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > LAUNCH_SECONDS)
                {
                    pathState = BasicStates.PARK;
                }
                break;

            case PARK:
                follower.followPath(park);
                if (!follower.isBusy()) pathState = BasicStates.IDLE;
                break;

            case IDLE:
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPedroPaths();
        follower.setStartingPose(startP);
    }

    @Override
    public void start()
    {
        opmodeTimer.resetTimer();
        pathState = BasicStates.INIT;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousStateMachine();

        telemetry.addData("path state:", pathState);
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("heading:", follower.getPose().getHeading());
        telemetry.update();
    }
}

enum BasicStates
{
    INIT,
    TO_ARTIFACT,
    TO_SCORING,
    PARK,
    IDLE
}
