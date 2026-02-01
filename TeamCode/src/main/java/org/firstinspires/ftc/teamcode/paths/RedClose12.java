package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedClose12 {
    public PathChain ScorePreloads;
    public PathChain IntakeSpike1;
    public PathChain ScoreSpike1;
    public PathChain AlignSpike2;
    public PathChain IntakeSpike2;
    public PathChain ScoreSpike2;
    public PathChain PrepAlignSpike3;
    public PathChain IntakeSpike3;
    public PathChain ScoreSpike3Part1;
    public PathChain ScoreSpike3Part2;
    public PathChain Leave;

    public RedClose12(Follower follower) {
        ScorePreloads = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(112.000, 136.500),
                                new Pose(112.000, 107.000),
                                new Pose(102.000, 104.000),
                                new Pose(89.000, 88.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        IntakeSpike1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(89.000, 88.000),
                                new Pose(89.000, 84.000),
                                new Pose(126.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        ScoreSpike1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(126.000, 84.000),
                                new Pose(89.000, 84.000),
                                new Pose(89.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51))

                .build();

        AlignSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.000, 88.000),

                                new Pose(89.000, 62.0)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(0))

                .build();

        IntakeSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.000, 62.0),

                                new Pose(126.000, 62.0)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        ScoreSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.000, 62.0),

                                new Pose(89.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51))
                .build();

        PrepAlignSpike3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.000, 88.000),

                                new Pose(89.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(-90))
                .build();

        IntakeSpike3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(89.000, 82.000),
                                new Pose(77.000, 35.000),
                                new Pose(100.000, 35.000),
                                new Pose(126.000, 36.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        ScoreSpike3Part1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.000, 36.000),

                                new Pose(94.000, 81.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-55))

                .build();

        ScoreSpike3Part2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.000, 81.000),

                                new Pose(89.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-55), Math.toRadians(51))

                .build();

        Leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.000, 88.000),

                                new Pose(99.000, 78.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(0))

                .build();
    }
}
  