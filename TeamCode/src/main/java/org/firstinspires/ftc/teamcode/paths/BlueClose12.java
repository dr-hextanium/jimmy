package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueClose12 {
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

    public BlueClose12(Follower follower) {
        ScorePreloads = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(32.000, 136.500),
                                new Pose(32.000, 107.000),
                                new Pose(42.000, 104.000),
                                new Pose(55.000, 88.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        IntakeSpike1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.000, 88.000),
                                new Pose(55.000, 84.000),
                                new Pose(18.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        ScoreSpike1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.000, 84.000),
                                new Pose(55.000, 84.000),
                                new Pose(55.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129))

                .build();

        AlignSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 88.000),

                                new Pose(55.000, 62.0)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(180))

                .build();

        IntakeSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 62.0),

                                new Pose(18.000, 62.0)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        ScoreSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.000, 62.0),

                                new Pose(55.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129))
                .build();

        PrepAlignSpike3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 88.000),

                                new Pose(55.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(270))
                .build();

        IntakeSpike3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.000, 82.000),
                                new Pose(67.000, 35.000),
                                new Pose(44.000, 35.000),
                                new Pose(18.000, 36.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        ScoreSpike3Part1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.000, 36.000),

                                new Pose(50.000, 81.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(235))

                .build();

        ScoreSpike3Part2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 81.000),

                                new Pose(55.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(129))

                .build();

        Leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 88.000),

                                new Pose(45.000, 78.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(0))

                .build();
    }
}
