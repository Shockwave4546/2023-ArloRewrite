package frc.robot

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.drivetrain.DriveSubsystem
import frc.robot.subsystems.elevator.ElevatorSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem
import frc.robot.subsystems.intakepivot.PivotIntakeDownCommand
import frc.robot.subsystems.shooter.ShooterSubsystem
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
    driveController.a().whileTrue(EndlessCommand({ ShooterSubsystem.runShooter(-Shooter.SPEED) }, { ShooterSubsystem.stopMotor() }, ShooterSubsystem))
    driveController.leftBumper().whileTrue(EndlessCommand({ ElevatorSubsystem.runMotors(Elevator.BELT_SPEED, Elevator.TRAP_SPEED) }, { ElevatorSubsystem.stopMotors() }, ElevatorSubsystem))
    driveController.rightBumper().whileTrue(EndlessCommand({ IntakeSubsystem.runIntake(Shooter.SPEED) }, { IntakeSubsystem.stopMotor() }, IntakeSubsystem))

    // Inverted commands
    driveController.x().whileTrue(EndlessCommand({ IntakeSubsystem.runIntake(-Shooter.SPEED) }, { IntakeSubsystem.stopMotor() }, IntakeSubsystem))
    driveController.b().whileTrue(EndlessCommand({ ElevatorSubsystem.runMotors(-Elevator.BELT_SPEED, -Elevator.TRAP_SPEED) }, { ElevatorSubsystem.stopMotors() }, ElevatorSubsystem))
  }

  private fun addDebugButtons() {
    Tabs.DEBUG.add("Pivot Intake Down", PivotIntakeDownCommand())
  }
}