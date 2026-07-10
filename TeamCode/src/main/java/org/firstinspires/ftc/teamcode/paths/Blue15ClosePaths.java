package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Blue15ClosePaths {
    public PathChain ScorePreloads;
    public PathChain IntakeSpike2;
    public PathChain ScoreSpike2;
    public PathChain OpenGate1;
    public PathChain IntakeGate1;
    public PathChain ScoreGate1;

    public Blue15ClosePaths(Follower follower) {
        ScorePreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(26.000, 130.000),

                                new Pose(52.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(137))
                .setReversed()
                .build();

        IntakeSpike2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000, 90.000),
                                new Pose(53.000, 69.000),
                                new Pose(60.000, 56.000),
                                new Pose(10.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();

        ScoreSpike2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.000, 60.000),
                                new Pose(22.000, 54.000),
                                new Pose(35.000, 55.000),
                                new Pose(52.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();

        OpenGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000, 90.000),
                                new Pose(34.000, 69.000),
                                new Pose(15.000, 69.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(90))

                .build();

        IntakeGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 69.000),
                                new Pose(19.000, 53.000),
                                new Pose(9.000, 47.000),
                                new Pose(8.000, 54.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(100))

                .build();

        ScoreGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(8.000, 54.000),
                                new Pose(39.000, 63.000),
                                new Pose(52.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(137))

                .build();
    }
}
