package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueClose12OldPaths {
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

    // Red shotAngle was 38 (51-13).
    // Mirrored for Blue: 180 - 38 = 142 degrees.
    double shotAngle = 180 - (51 - 13);

    public BlueClose12OldPaths(Follower follower) {
        ScorePreloads = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(32.000, 136.500),
                                new Pose(32.000, 107.000),
                                new Pose(42.000, 104.000),
                                new Pose(55.000, 88.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .setHeadingConstraint(0.1)
                .build();

        IntakeSpike1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.000, 88.000),
                                new Pose(55.000, 84.000),
                                new Pose(14.000, 85.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ScoreSpike1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(14.000, 85.000),
                                new Pose(55.000, 84.000),
                                new Pose(55.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shotAngle - 5))
                .setHeadingConstraint(0.1)
                .build();

        AlignSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 88.000),
                                new Pose(55.000, 61.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(shotAngle), Math.toRadians(180))
                .build();

        IntakeSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 61.000),
                                new Pose(7.500, 61.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        ScoreSpike2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(7.500, 61.000),
                                new Pose(31.000, 52.000),
                                new Pose(55.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shotAngle))
                .setHeadingConstraint(0.1)
                .build();

        PrepAlignSpike3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 88.000),
                                new Pose(55.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(shotAngle + 2), Math.toRadians(270))
                .build();

        IntakeSpike3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.000, 82.000),
                                new Pose(67.000, 29.000),
                                new Pose(44.000, 35.000),
                                new Pose(7.000, 38.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        ScoreSpike3Part1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(7.000, 38.000),
                                new Pose(50.000, 81.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(235)) // Mirrored from -55
                .build();

        ScoreSpike3Part2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 81.000),
                                new Pose(55.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(shotAngle + 2))
                .setHeadingConstraint(0.008)
                .build();

        Leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 88.000),
                                new Pose(55.000, 68.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(shotAngle + 2), Math.toRadians(180))
                .build();
    }
}