package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPath {

    public PathBuilder builder;
    public PathChain paths;
    public GeneratedPath(Follower follower)
    {
        builder = new PathBuilder(follower);

         paths = builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Pose(56.000, 9.000),
                                new Pose(58.000, 28.000),
                                new Pose(42.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
    }

    public PathChain getPath()
    {
        return paths;
    }
}