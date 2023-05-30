package frc.robot.subsystems.elevator

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Elevator

object ElevatorSubsystem : SubsystemBase() {
  private val beltMotor = WPI_VictorSPX(Elevator.BELT_MOTOR_ID)
  private val trapMotor = WPI_VictorSPX(Elevator.TRAP_MOTOR_ID)

  fun runBelt(speed: Double) = beltMotor.set(speed)

  fun runTrap(speed: Double) = trapMotor.set(speed)

  fun runMotors(beltSpeed: Double, trapSpeed: Double) {
    runBelt(beltSpeed)
    runTrap(trapSpeed)
  }

  fun stopMotors() {
    beltMotor.stopMotor()
    trapMotor.stopMotor()
  }
}