package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Elevator

object ShooterSubsystem : SubsystemBase() {
  private val shooterMotor = WPI_VictorSPX(Elevator.BELT_MOTOR_ID)

  fun runShooter(speed: Double) = shooterMotor.set(speed)

  fun stopMotor() {
    shooterMotor.stopMotor()
  }
}