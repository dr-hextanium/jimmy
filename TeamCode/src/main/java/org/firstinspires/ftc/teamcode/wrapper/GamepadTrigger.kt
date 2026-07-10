package org.firstinspires.ftc.teamcode.wrapper

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys


class GamepadTrigger(private val m_gamepad: GamepadEx, val threshold: Double = 0.3, vararg triggers: GamepadKeys.Trigger) : Trigger() {
	private val m_triggers: Array<GamepadKeys.Trigger> = triggers as Array<GamepadKeys.Trigger>

    fun whenReleased(command: Command, interruptible: Boolean): Trigger {
        CommandScheduler.getInstance().addButton(object : Runnable {
            private var m_pressedLast = get()

            override fun run() {
                val pressed = get()

                if (m_pressedLast && !pressed) {
                    command.schedule(interruptible)
                }

                m_pressedLast = pressed
            }
        })
        return this
    }

	override fun get(): Boolean {
		var res = true
		for (trigger in m_triggers) res = res && m_gamepad.getTrigger(trigger) > threshold
		return res
	}
}