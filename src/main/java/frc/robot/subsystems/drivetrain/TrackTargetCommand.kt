package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Drive
import frc.robot.RobotContainer
import frc.robot.Tabs

class TrackTargetCommand : CommandBase() {
  companion object {
    private val FORWARD_CONTROLLER = ProfiledPIDController(Drive.LINEAR_P, 0.0, Drive.LINEAR_D,
      TrapezoidProfile.Constraints(1.0, 0.5)
    )
    private val TURN_CONTROLLER = PIDController(Drive.ANGULAR_P, 0.0, Drive.ANGULAR_D)
  }

  init {
    addRequirements(DriveSubsystem)
    Tabs.DEBUG.add("Forward Controller", FORWARD_CONTROLLER)
    Tabs.DEBUG.add("Turn Controller", TURN_CONTROLLER)
  }

  override fun initialize() = DriveSubsystem.stop()

  override fun execute() {
    val result = RobotContainer.camera.latestResult
    if (!result.hasTargets()) return
    val bestTarget = result.bestTarget
//    val range = PhotonUtils.calculateDistanceToTargetMeters(
//      Drive.CAMERA_HEIGHT_METERS,
//      Drive.TARGET_HEIGHT_METERS,
//      Drive.CAMERA_PITCH_RADIANS,
//      Units.degreesToRadians(bestTarget.pitch)
//    )
    val range = bestTarget.bestCameraToTarget.x
    val forwardSpeed = -FORWARD_CONTROLLER.calculate(range, Drive.GOAL_RANGE_DISTANCE_METERS)
    val rotationSpeed = -TURN_CONTROLLER.calculate(bestTarget.yaw, 0.0)
    DriveSubsystem.arcadeDrive(-forwardSpeed, rotationSpeed)
  }

  override fun isFinished() = FORWARD_CONTROLLER.atSetpoint() && TURN_CONTROLLER.atSetpoint()

  override fun end(interrupted: Boolean) = DriveSubsystem.stop()
}