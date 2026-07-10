package org.firstinspires.ftc.teamcode.command.intake

import com.arcrobotics.ftclib.command.ParallelCommandGroup

class IntakeWithGateClosed : ParallelCommandGroup(IntakeIn(0.8), CloseGate())