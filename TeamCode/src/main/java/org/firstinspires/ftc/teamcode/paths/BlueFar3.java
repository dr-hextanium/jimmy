package org.firstinspires.ftc.teamcode.paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueFar3 {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;

    public BlueFar3(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 7.500),

                                new Pose(64.000, 25.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(118))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(64.000, 25.000),

                                new Pose(57.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.000, 36.000),

                                new Pose(13.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.000, 36.000),

                                new Pose(55.000, 29.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(170))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 29.000),

                                new Pose(64.000, 25.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(170), Math.toRadians(118))

                .build();
    }
}
