package frc.robot

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.drivetrain.DriveSubsystem
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem
import frc.robot.subsystems.intakepivot.PivotIntakeDownCommand
import org.photonvision.PhotonCamera

object RobotContainer {
  val driveController = CommandXboxController(DIO.DRIVE_CONTROLLER_PORT)
  val camera = PhotonCamera(DIO.CAMERA_NAME)

  init {
    DriveSubsystem
    IntakePivotSubsystem
    configureBindings()
    addDebugButtons()
  }

  private fun configureBindings() {
    driveController.povUp().whileTrue(EndlessCommand({ IntakePivotSubsystem.runMotor(IntakePivot.SPEED) }, { IntakePivotSubsystem.stopMotor() }, IntakePivotSubsystem))
    driveController.povDown().whileTrue(EndlessCommand({ IntakePivotSubsystem.runMotor(-IntakePivot.SPEED) }, { IntakePivotSubsystem.stopMotor() }, IntakePivotSubsystem))
  }

  private fun addDebugButtons() {
    Tabs.DEBUG.add("Pivot Intake Down", PivotIntakeDownCommand())
  }
}