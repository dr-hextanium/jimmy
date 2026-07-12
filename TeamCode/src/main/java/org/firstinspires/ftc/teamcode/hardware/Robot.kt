package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannelImpl
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystem.Launcher
import org.firstinspires.ftc.teamcode.hardware.subsystem.Turret
import org.firstinspires.ftc.teamcode.hardware.wrapper.BeamBreak
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


object Robot : ISubsystem {
	val scheduler: CommandScheduler
		get() = CommandScheduler.getInstance()

	lateinit var hubs: List<LynxModule>

	lateinit var telemetry: MultipleTelemetry
	lateinit var hw: HardwareMap

	lateinit var gamepad1: GamepadEx
	lateinit var gamepad2: GamepadEx

	lateinit var follower: Follower

	val pose
		get() = follower.pose

	object Subsystems {
		lateinit var intake: Intake
		lateinit var turret: Turret
		lateinit var launcher: Launcher

		fun all() = listOf(intake, turret, launcher)
	}

	object Motors {
		object Intake { lateinit var motor: DcMotorEx }

		object Turret { lateinit var motor: DcMotorEx }

		object Launcher {
			lateinit var leftMotor: DcMotorEx
			lateinit var rightMotor: DcMotorEx
		}

		fun all() = listOf(
			Intake.motor,
			Turret.motor,
			Launcher.rightMotor,
			Launcher.leftMotor
		)
	}

    object Servos {
        object Intake { lateinit var gate: Servo }

        object Launcher {
            lateinit var hood: Servo
        }

        fun all() = listOf(
            Servos.Intake.gate,
            Servos.Launcher.hood
        )
    }

    object DigitalDevices {
        object Intake {
            lateinit var bottomBeamBreak: BeamBreak
            lateinit var middleBeamBreak: BeamBreak
            lateinit var topBeamBreak: BeamBreak
        }
    }

    object AnalogDevices {
        object Turret {
            lateinit var encoder12Tooth: AnalogInput
            lateinit var encoder13Tooth: AnalogInput
        }
    }

	fun init(hw: HardwareMap, telemetry: Telemetry, gamepad1: Gamepad, gamepad2: Gamepad) {
		Robot.telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
		Robot.hw = hw

		Robot.telemetry.msTransmissionInterval = 10

		hubs = hw.getAll(LynxModule::class.java)
		hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

		Robot.gamepad1 = GamepadEx(gamepad1)
		Robot.gamepad2 = GamepadEx(gamepad2)

		run {
			Motors.Intake.motor = hw[Names.Motors.Intake.motor] as DcMotorEx
			Motors.Turret.motor = hw[Names.Motors.Turret.motor] as DcMotorEx
			Motors.Launcher.leftMotor = hw[Names.Motors.Launcher.leftMotor] as DcMotorEx
			Motors.Launcher.rightMotor = hw[Names.Motors.Launcher.rightMotor] as DcMotorEx

            Servos.Intake.gate = hw[Names.Servos.Intake.servo] as Servo
            Servos.Launcher.hood = hw[Names.Servos.Launcher.servo] as Servo

            DigitalDevices.Intake.bottomBeamBreak = BeamBreak(hw.digitalChannel[Names.DigitalDevices.Intake.bottomBeamBreak] as DigitalChannelImpl)
            DigitalDevices.Intake.middleBeamBreak = BeamBreak(hw.digitalChannel[Names.DigitalDevices.Intake.middleBeamBreak] as DigitalChannelImpl)
            DigitalDevices.Intake.topBeamBreak = BeamBreak(hw.digitalChannel[Names.DigitalDevices.Intake.topBeamBreak] as DigitalChannelImpl)

            AnalogDevices.Turret.encoder12Tooth = hw[Names.AnalogDevices.Turret.encoder12Tooth] as AnalogInput
            AnalogDevices.Turret.encoder13Tooth = hw[Names.AnalogDevices.Turret.encoder13Tooth] as AnalogInput
		}

        follower = Constants.createFollower(hw)
        follower.setStartingPose(Pose(0.0, 0.0, 0.0))
        follower.update()

		Subsystems.intake = Intake(
            Motors.Intake.motor,
            Servos.Intake.gate,
            DigitalDevices.Intake.bottomBeamBreak,
            DigitalDevices.Intake.middleBeamBreak,
            DigitalDevices.Intake.topBeamBreak
        )
		Subsystems.turret = Turret(Motors.Turret.motor, AnalogDevices.Turret.encoder12Tooth, AnalogDevices.Turret.encoder13Tooth)
		Subsystems.launcher = Launcher(Motors.Launcher.leftMotor, Motors.Launcher.rightMotor, Servos.Launcher.hood)

		scheduler.registerSubsystem(*Subsystems.all().toTypedArray())

		reset()
	}

	override fun reset() {
		scheduler.reset()
		Subsystems.all().forEach { it.reset() }
	}

	override fun read() {
		follower.update()

		Subsystems.all().forEach { it.read() }
	}

	override fun update() {
		scheduler.run()
		Subsystems.all().forEach { it.update() }
	}

	override fun write() {
		Subsystems.all().forEach { it.write() }
	}
}