package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueClose12Paths {
    public PathChain ScorePreloads;
    public PathChain IntakeSpike2;
    public PathChain OpenGate;
    public PathChain PrepScoreSpike2;
    public PathChain ScoreSpike2;
    public PathChain IntakeSpike1;
    public PathChain PrepScoreSpike1;
    public PathChain ScoreSpike1;
    public PathChain IntakeSpike3;
    public PathChain PrepScoreSpike3;
    public PathChain ScoreSpike3;

    public BlueClose12Paths(Follower follower) {
        ScorePreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(26.000, 130.000),

                                new Pose(52.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(137))

                .build();

        IntakeSpike2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000, 90.000),
                                new Pose(66.788, 56.831),
                                new Pose(41.169, 59.519),
                                new Pose(10.071, 59.409)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        OpenGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.071, 59.409),
                                new Pose(35.666, 59.443),
                                new Pose(16.198, 64.941)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PrepScoreSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.198, 64.941),

                                new Pose(52.743, 76.864)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setReversed()
                .build();

        ScoreSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.743, 76.864),

                                new Pose(52.149, 90.149)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();

        IntakeSpike1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.149, 90.149),
                                new Pose(45.379, 83.528),
                                new Pose(16.628, 83.650)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        PrepScoreSpike1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.628, 83.650),

                                new Pose(46.449, 88.644)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        ScoreSpike1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.449, 88.644),

                                new Pose(52.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();

        IntakeSpike3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.000, 90.000),
                                new Pose(63.704, 36.663),
                                new Pose(50.574, 35.879),
                                new Pose(10.752, 36.204)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        PrepScoreSpike3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.752, 36.204),
                                new Pose(44.299, 49.474),
                                new Pose(50.663, 84.303)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        ScoreSpike3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.663, 84.303),

                                new Pose(60.149, 102.474)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))

                .build();
    }
}
