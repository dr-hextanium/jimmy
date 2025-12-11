package org.firstinspires.ftc.teamcode.programs

import com.bylazar.telemetry.PanelsTelemetry
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.ifHuh
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.scope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Launcher
import org.firstinspires.ftc.teamcode.subsystems.Limelight
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import kotlin.math.PI

val DriverControlled = Mercurial.teleop {
    val telemetry = PanelsTelemetry.telemetry

    val drivetrain = Drivetrain(hardwareMap, telemetry)
    val intake = Intake(hardwareMap)
    val launcher = Launcher(hardwareMap)
    val transfer = Transfer(hardwareMap)
    val limelight = Limelight(hardwareMap)

    // ill do ts later
    val allianceHeadingLock = PI / 2

    bindExec(
        { true },
        loop(
            ifHuh(
                // auto aim
                { gamepad1.triangle },
                drivetrain.driveHeadingLock(
                    { -gamepad1.left_stick_y.toDouble() },
                    { -gamepad1.left_stick_x.toDouble() },
                    {
                        val txRad = Math.toRadians(limelight.getAngleToGoal())
                        drivetrain.follower.pose.heading - txRad
                    }
                )
            ).elseIfHuh(
                // heading lock
                { gamepad1.right_stick_button },
                drivetrain.driveHeadingLock(
                    { -gamepad1.left_stick_y.toDouble() },
                    { -gamepad1.left_stick_x.toDouble() },
                    { allianceHeadingLock }
                )
            ).elseHuh(
                // drive
                drivetrain.drive(
                    { -gamepad1.left_stick_y.toDouble() },
                    { -gamepad1.left_stick_x.toDouble() },
                    { -gamepad1.right_stick_x.toDouble() }
                )
            )
        )
    )

    bindSpawn(
        risingEdge { gamepad1.left_stick_button },
        scope {
            // vision to var for frame
            val pose by value { limelight.getRelocalizationPose() }

            ifHuh(
                { pose != null },
                // relocalize and vribrate
                sequence(
                    drivetrain.relocalize { pose!! },
                    exec { gamepad1.rumble(500) }
                )
            )
        }
    )

    bindExec(
        { true },
        loop(
            ifHuh(
                { gamepad1.right_trigger > 0.5 },
                intake.intake
            ).elseIfHuh(
                { gamepad1.left_trigger > 0.5 },
                intake.outtake
            ).elseHuh(
                intake.stop
            )
        )
    )

    bindExec(
        { true },
        loop(
            ifHuh(
                { gamepad1.square },
                launcher.spinUp { limelight.getDistanceFromGoal() }
            ).elseHuh(
                launcher.stop
            )
        )
    )

    bindExec(
        { true },
        loop(
            ifHuh(
                // must be ready to fire
                { gamepad1.circle && launcher.isReady },
                transfer.feed
            ).elseIfHuh(
                { gamepad1.dpad_down },
                transfer.reverse
            ).elseHuh(
                transfer.stop
            )
        )
    )

    bindExec(
        { true },
        loop(
            ifHuh(
                { gamepad1.triangle || gamepad1.square },
                limelight.setPipeline(Limelight.PIPELINE_SCORING)
            ).elseHuh(
                limelight.setPipeline(Limelight.PIPELINE_TAGS)
            )
        )
    )
}