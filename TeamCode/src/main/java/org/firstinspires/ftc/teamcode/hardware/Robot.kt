package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.pedropathing.follower.Follower
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake
import org.firstinspires.ftc.teamcode.hardware.subsystem.Launcher
import org.firstinspires.ftc.teamcode.hardware.subsystem.Transfer
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

object Robot : ISubsystem {
	val scheduler: CommandScheduler
		get() = CommandScheduler.getInstance()

	lateinit var hubs: List<LynxModule>

	lateinit var telemetry: MultipleTelemetry
	lateinit var hw: HardwareMap

	lateinit var gamepad1: GamepadEx
	lateinit var gamepad2: GamepadEx

	lateinit var voltageSensor: Iterator<VoltageSensor>
	var voltageTimer = ElapsedTime()
	var voltage: Double = 0.0

	lateinit var follower: Follower

	val pose
		get() = follower.pose

	object Subsystems {
		lateinit var intake: Intake
		lateinit var transfer: Transfer
		lateinit var launcher: Launcher

		fun all() = listOf(intake, transfer, launcher)
	}

	object Motors {
		object Intake { lateinit var motor: DcMotorEx }

		object Transfer { lateinit var motor: DcMotorEx }

		object Launcher {
			lateinit var leftMotor: DcMotorEx
			lateinit var rightMotor: DcMotorEx
		}

		fun all() = listOf(
			Intake.motor,
			Transfer.motor,
			Launcher.rightMotor,
			Launcher.leftMotor
		)
	}

	fun init(hw: HardwareMap, telemetry: Telemetry, gamepad1: Gamepad, gamepad2: Gamepad) {
		Robot.telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
		Robot.hw = hw

		Robot.telemetry.msTransmissionInterval = 11


		hubs = hw.getAll(LynxModule::class.java)
		hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

		voltageSensor = hw.voltageSensor.iterator()
		voltageTimer.reset()

		Robot.gamepad1 = GamepadEx(gamepad1)
		Robot.gamepad2 = GamepadEx(gamepad2)

		run {
			Motors.Intake.motor = hw[Names.Motors.Intake.motor] as DcMotorEx
			Motors.Transfer.motor = hw[Names.Motors.Transfer.motor] as DcMotorEx
			Motors.Launcher.leftMotor = hw[Names.Motors.Launcher.leftMotor] as DcMotorEx
			Motors.Launcher.rightMotor = hw[Names.Motors.Launcher.rightMotor] as DcMotorEx
		}

		val limelight = hw["limelight"] as Limelight3A

		follower = Constants.createFollower(hw)

		Subsystems.intake = Intake(Motors.Intake.motor)
		Subsystems.transfer = Transfer(Motors.Transfer.motor)
		Subsystems.launcher = Launcher(Motors.Launcher.leftMotor, Motors.Launcher.rightMotor)

		scheduler.registerSubsystem(*Subsystems.all().toTypedArray())

		reset()
	}

	override fun reset() {
		scheduler.reset()
		Subsystems.all().forEach { it.reset() }
	}

	override fun read() {
		follower.update()

		if (voltageTimer.milliseconds() > 100.0 && voltageSensor.hasNext()) {
			voltage = voltageSensor.next().voltage
		}

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