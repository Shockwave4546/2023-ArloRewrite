package frc.robot

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.robot.shuffleboard.AdjustableDouble

/**
 * Even though ShuffleboardTabs technically aren't stateless objects, it's more
 * convenient to place them in Constants.kt
 */
class Tabs {
  companion object {
    val DIO = Shuffleboard.getTab("DIO")!!
    val DEBUG = Shuffleboard.getTab("Debug")!!
  }
}

class DIO {
  companion object {
    const val DRIVE_CONTROLLER_PORT = 0
    const val CAMERA_NAME = "Camera"
  }
}

class Drive {
  companion object {
    private const val QUAD_ENCODER_PULSES_PER_REVOLUTION = 2048.0
    private val WHEEL_DIAMETER_METERS = Units.inchesToMeters(6.0)
    val DISTANCE_PER_PULSE = (Math.PI * WHEEL_DIAMETER_METERS) / QUAD_ENCODER_PULSES_PER_REVOLUTION
    val TRACK_WIDTH_METERS = Units.inchesToMeters(24.0)
    val CAMERA_TO_POSE = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d())

    val LEFT_ENCODER = arrayOf(0, 1)
    val RIGHT_ENCODER = arrayOf(2, 3)

    const val FRONT_LEFT_ID = 7
    const val FRONT_RIGHT_ID = 8
    const val BACK_LEFT_ID = 9
    const val BACK_RIGHT_ID = 10

    val CAMERA_HEIGHT_METERS = Units.inchesToMeters(45.0)
    val TARGET_HEIGHT_METERS = Units.feetToMeters(3.5)
    val CAMERA_PITCH_RADIANS = Units.degreesToRadians(0.0)
    val GOAL_RANGE_DISTANCE_METERS = Units.feetToMeters(2.25)

    const val LINEAR_P = 0.26
    const val LINEAR_D = 0.05

    const val ANGULAR_P = 0.05
    const val ANGULAR_D = 0.0
  }
}

class IntakePivot {
  companion object {
    const val MOTOR_ID = 6
    const val SPEED = 0.7
  }
}

class Intake {
  companion object {
    const val MOTOR_ID = 5
    const val SPEED = 0.85
  }
}

class Elevator {
  companion object {
    const val BELT_MOTOR_ID = 1
    const val BELT_SPEED = 0.6
    const val TRAP_MOTOR_ID = 2
    const val TRAP_SPEED = 0.75
  }
}

class Shooter {
  companion object {
    const val MOTOR_ID = 3
  }
}