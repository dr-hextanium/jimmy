package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedClose9Gate {
    public PathChain ScorePreloads;
    public PathChain IntakeSpike1;
    public PathChain OpenGate;
    public PathChain ScoreSpike1;
    public PathChain AlignSpike2;
    public PathChain IntakeSpike2;
    public PathChain ScoreSpike2;
    public PathChain Leave;

    public RedClose9Gate(Follower follower) {
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
                                new Pose(130.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        OpenGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(130.000, 84.000),
                                new Pose(113.000, 68.000),
                                new Pose(133.000, 70.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        ScoreSpike1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(133.000, 70.000),
                                new Pose(89.000, 84.000),
                                new Pose(89.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51))

                .build();

        AlignSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.000, 88.000),

                                new Pose(89.000, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(0))

                .build();

        IntakeSpike2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.000, 63.000),

                                new Pose(138.000, 63.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        ScoreSpike2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(138.000, 63.000),
                                new Pose(108.000, 66.000),
                                new Pose(89.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51))

                .build();

        Leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.000, 88.000),

                                new Pose(82.000, 65.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(0))
                .build();
    }
}
