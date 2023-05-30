package frc.robot

import com.pathplanner.lib.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.drivetrain.DriveSubsystem
import frc.robot.subsystems.drivetrain.TrackTargetCommand

object Robot : TimedRobot() {
  override fun robotInit() {
    RobotContainer
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()
  }

  override fun disabledInit() {

  }

  override fun disabledPeriodic() {

  }

  override fun autonomousInit() {

  }

  override fun autonomousPeriodic() {

  }

  override fun teleopInit() {
    DriveSubsystem.initTeleop(RobotContainer.driveController)
    RobotContainer.driveController.a().onTrue {
      val result = RobotContainer.camera.latestResult
      if (!result.hasTargets()) return@onTrue setOf(DriveSubsystem)
      val bestTarget = result.bestTarget
      val range = bestTarget.bestCameraToTarget.x
      // If the intended range is already met, then don't proceed further.
      if (Drive.GOAL_RANGE_DISTANCE_METERS >= range) return@onTrue setOf(DriveSubsystem)
      DriveSubsystem.loadRelativeTrajectoryToRamseteCommand(PathConstraints(3.0, 1.0), Pose2d(range - Drive.GOAL_RANGE_DISTANCE_METERS, 0.0, Rotation2d())).schedule()
      return@onTrue setOf(DriveSubsystem)
    }
  }

  override fun teleopPeriodic() {

  }

  override fun testInit() {
    CommandScheduler.getInstance().cancelAll()
  }

  override fun testPeriodic() {

  }

  override fun simulationInit() {

  }

  override fun simulationPeriodic() {

  }
}