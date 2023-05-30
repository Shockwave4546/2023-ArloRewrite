package frc.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Elevator

object IntakeSubsystem : SubsystemBase() {
  private val intakeMotor = WPI_VictorSPX(Elevator.BELT_MOTOR_ID)

  fun runIntake(speed: Double) = intakeMotor.set(speed)

  fun stopMotor() {
    intakeMotor.stopMotor()
  }
}