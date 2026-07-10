package org.firstinspires.ftc.teamcode.command.turret

import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.hardware.Robot

class StopAimingAtGoal : InstantCommand({ Robot.Subsystems.turret.aimAtGoal = false })
