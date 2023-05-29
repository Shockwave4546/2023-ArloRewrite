package frc.robot.subsystems.drivetrain

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

class TankDriveCommand(private val controller: CommandXboxController) : CommandBase() {
  init {
    addRequirements(DriveSubsystem)
  }

  override fun initialize() = DriveSubsystem.stop()

  override fun execute() = DriveSubsystem.tankDrive(controller.leftY, controller.rightY)

  override fun end(interrupted: Boolean) = DriveSubsystem.stop()
}