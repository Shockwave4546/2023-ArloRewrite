package frc.robot.subsystems.intakepivot

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.IntakePivot
import kotlin.math.abs

class PivotIntakeDownCommand : CommandBase() {
  init {
    addRequirements(IntakePivotSubsystem)
  }

  override fun initialize() {
    IntakePivotSubsystem.stopMotor()
    IntakePivotSubsystem.resetEncoder()
  }

  override fun execute() = IntakePivotSubsystem.runMotor(-(IntakePivot.SPEED - 0.1))

  /**
   * 1 spin roughly is 20 "positions"
   */
  override fun isFinished() = abs(IntakePivotSubsystem.getEncoderPosition()) >= 187

  override fun end(interrupted: Boolean) = IntakePivotSubsystem.stopMotor()
}