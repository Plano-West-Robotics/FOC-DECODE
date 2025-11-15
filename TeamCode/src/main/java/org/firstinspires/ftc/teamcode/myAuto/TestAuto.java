package org.firstinspires.ftc.teamcode.myAuto;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.examplePedroAuto.BasicStates;

@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private BasicStates pathState;

    private final Pose startP = new Pose(56, 9); // the bot's starting location on the field
    private final Pose pickup1_1P = new Pose(62, 64); // intermediate curve point
    private final Pose pickup1_2P = new Pose(37, 84); //final curve point (ends by facing artifacts)
    private final Pose pickup1_3P = new Pose(16, 84); // runs over artifacts

    private final Pose launch = new Pose(47, 96); // line end point

    private final Pose parkP = new Pose(53, 64); // intermediate curve point
    private final Pose parkP2 = new Pose(38, 34); // parking spot

    private Path goToArtifacts;
    private PathChain goToScoring;
    private Path park;

    public static final int INIT_SECONDS = 1;
    public static final int LAUNCH_SECONDS = 3;

    public void runOpMode() {

    }

    // make paths with different start and end positions and angles
    public void buildPedroPaths() {
        goToArtifacts = new Path(new BezierCurve(startP, pickup1_1P, pickup1_2P));
        goToArtifacts.setLinearHeadingInterpolation(NINETY, ONEEIGHTY);

        goToScoring = follower.pathBuilder()
                .addPath(new BezierLine(pickup1_2P, pickup1_3P))
                .setConstantHeadingInterpolation(ONEEIGHTY)
                .addPath(new BezierLine(pickup1_3P, launch))
                .setLinearHeadingInterpolation(ONEEIGHTY, Math.toRadians(135))
                .build();

        park = new Path(new BezierCurve(launch, parkP, parkP2));
        park.setLinearHeadingInterpolation(Math.toRadians(135), NINETY);
    }

    public void setPathState(BasicStates pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousStateMachine() {
        switch(pathState) {
            // when starting you may want to wait a second to reposition servos and motors
            case INIT:
                if (pathTimer.getElapsedTimeSeconds() >= INIT_SECONDS) setPathState(BasicStates.TO_ARTIFACT);
                break;

            case TO_ARTIFACT:
                follower.followPath(goToArtifacts, true); // holdEnd means it will stay at the point after reaching it

                // move to the next state when the path has completed
                if (!follower.isBusy()) setPathState(BasicStates.TO_SCORING);
                break;

            case TO_SCORING:
                follower.followPath(goToScoring, true);

                if (!follower.isBusy()) setPathState(BasicStates.AT_SCORING);
                break;

            case AT_SCORING:
                // 3 seconds after the robot has reached the end position
        }
    }
}