package frc.robot

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * Intended for use in conjunction with [edu.wpi.first.wpilibj2.command.button.Trigger]
 * Ex: #a().whileTrue(EndlessCommand(startMotor(), stopMotor(), ...))
 */
class EndlessCommand(private val toRun: () -> Unit, private val onEnd: () -> Unit, subsystem: SubsystemBase) : CommandBase() {
  init {
    addRequirements(subsystem)
  }

  override fun execute() = toRun()

  override fun end(interrupted: Boolean) = onEnd()
}