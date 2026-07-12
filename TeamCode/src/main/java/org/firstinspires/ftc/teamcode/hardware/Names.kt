package org.firstinspires.ftc.teamcode.hardware

object Names {
	object Motors {
		object Intake {
			const val motor = "intake"
		}

		object Turret {
			const val motor = "turret"
		}

		object Launcher {
			const val leftMotor = "sl"
			const val rightMotor = "sr"
		}

		object Drivetrain {
			const val frontRight = "fr"
			const val frontLeft = "fl"
			const val backRight = "br"
			const val backLeft = "bl"
		}
	}

    object Servos {
        object Intake {
            const val servo = "gate"
        }

        object Launcher {
            const val servo = "hood"
        }
    }

    object DigitalDevices {
        object Intake {
            const val bottomBeamBreak = "bbb"
            const val middleBeamBreak = "mbb"
            const val topBeamBreak = "tbb"
        }
    }

    object AnalogDevices {
        object Turret {
            const val encoder12Tooth = "te12"
            const val encoder13Tooth = "te13"
        }
    }
}