package frc.robot.subsystems.intakepivot

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.IntakePivot

object IntakePivotSubsystem : SubsystemBase() {
  private val intakePivotMotor = CANSparkMax(IntakePivot.MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val intakePivotEncoder = intakePivotMotor.encoder

  fun runMotor(speed: Double) = intakePivotMotor.set(speed)

  fun getEncoderPosition() = intakePivotEncoder.position

  @Suppress("HasPlatformType")
  fun resetEncoder() = intakePivotEncoder.setPosition(0.0)

  fun stopMotor() = intakePivotMotor.stopMotor()
}