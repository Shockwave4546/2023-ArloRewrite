package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
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
    RobotContainer.driveController.a().whileTrue(TrackTargetCommand())
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