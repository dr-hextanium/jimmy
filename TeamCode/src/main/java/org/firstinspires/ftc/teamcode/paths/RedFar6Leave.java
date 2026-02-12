package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedFar6Leave {
    public PathChain LaunchPreloads;
    public PathChain IntakeHumanPlayer;
    public PathChain GoBackToShoot;
    public PathChain ShootHumanPlayer;
    public PathChain Leave;

    public RedFar6Leave(Follower follower) {
        LaunchPreloads = follower.pathBuilder().addPath(
            new BezierLine(
                    new Pose(89.000, 7.500),

        new Pose(74.000, 22.000)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

        .build();

        IntakeHumanPlayer = follower.pathBuilder().addPath(
            new BezierLine(
                    new Pose(74.000, 22.000),

        new Pose(135.000, 8.000)
        )
        ).setTangentHeadingInterpolation()

        .build();

        GoBackToShoot = follower.pathBuilder().addPath(
            new BezierLine(
                    new Pose(135.000, 8.000),

        new Pose(81.000, 21.000)
        )
        ).setTangentHeadingInterpolation()
            .build();

        ShootHumanPlayer = follower.pathBuilder().addPath(
            new BezierLine(
                    new Pose(81.000, 21.000),

        new Pose(74.000, 22.000)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(-13), Math.toRadians(65))

        .build();

        Leave = follower.pathBuilder().addPath(
            new BezierLine(
                    new Pose(74.000, 22.000),

        new Pose(103.000, 10.000)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))

        .build();
    }
}
