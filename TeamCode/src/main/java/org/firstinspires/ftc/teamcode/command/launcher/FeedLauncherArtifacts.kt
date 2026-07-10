package org.firstinspires.ftc.teamcode.command.launcher

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.command.intake.IntakeIn
import org.firstinspires.ftc.teamcode.command.intake.OpenGate

class FeedLauncherArtifacts : ParallelCommandGroup(IntakeIn(), OpenGate())