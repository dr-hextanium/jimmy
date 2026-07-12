package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.Names
import kotlin.math.abs

/**
 * Open-loop flywheel characterization: dial motor power up/down and read the resulting velocity
 * (ticks/sec) of each launcher motor. Use it to map power -> RPM/TPS and to sanity-check that both
 * motors spin together (they drive one flywheel 1:1). Surgical -- touches only the two launcher
 * motors (no PID, no subsystem, no drivetrain), and starts at zero power.
 *
 * Controls (gamepad1): dpad up/down = +/- 0.05 power, right bumper = stop.
 */
@TeleOp(name = "Launcher RPM (open loop)", group = "Debug")
class RPMTest : OpMode() {
    private val left by lazy { hardwareMap.get(DcMotorEx::class.java, Names.Motors.Launcher.leftMotor) }
    private val right by lazy { hardwareMap.get(DcMotorEx::class.java, Names.Motors.Launcher.rightMotor) }

    private var power = 0.0
    private var lastUp = false
    private var lastDown = false

    override fun init() {
        // Match the Launcher subsystem's directions so both wheels drive the flywheel the same way.
        left.direction = DcMotorSimple.Direction.FORWARD
        right.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        // Rising-edge stepping so a held dpad doesn't ramp uncontrollably.
        if (gamepad1.dpad_up && !lastUp) power += 0.05
        if (gamepad1.dpad_down && !lastDown) power -= 0.05
        lastUp = gamepad1.dpad_up
        lastDown = gamepad1.dpad_down
        if (gamepad1.right_bumper) power = 0.0

        power = Range.clip(power, 0.0, 1.0)

        left.power = power
        right.power = power

        telemetry.addData("commanded power", "%.2f", power)
        telemetry.addData("left vel (tps)", "%.0f", left.velocity)
        telemetry.addData("right vel (tps)", "%.0f", right.velocity)
        telemetry.addData("avg |vel| (tps)", "%.0f", (abs(left.velocity) + abs(right.velocity)) / 2.0)
        telemetry.addLine("dpad up/down: +/-0.05 power | right bumper: stop")
    }

    override fun stop() {
        left.power = 0.0
        right.power = 0.0
    }
}
