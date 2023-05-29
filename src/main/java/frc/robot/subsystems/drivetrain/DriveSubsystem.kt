package frc.robot.subsystems.drivetrain

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Drive
import frc.robot.RobotContainer
import frc.robot.Tabs
import frc.robot.shuffleboard.AdjustableSpeed
import frc.robot.shuffleboard.Pos2D
import org.photonvision.PhotonUtils

object DriveSubsystem : SubsystemBase() {
  private val frontLeftMotor = WPI_VictorSPX(Drive.FRONT_LEFT_ID)
  private val frontRightMotor = WPI_VictorSPX(Drive.FRONT_RIGHT_ID)
  private val backLeftMotor = WPI_VictorSPX(Drive.BACK_LEFT_ID)
  private val backRightMotor = WPI_VictorSPX(Drive.BACK_RIGHT_ID)
  private val leftMotors = MotorControllerGroup(frontLeftMotor, backLeftMotor)
  private val rightMotors = MotorControllerGroup(frontRightMotor, backRightMotor)
  private val leftEncoder = Encoder(Drive.LEFT_ENCODER[0], Drive.LEFT_ENCODER[1])
  private val rightEncoder = Encoder(Drive.RIGHT_ENCODER[0], Drive.RIGHT_ENCODER[1])
  private val drive = DifferentialDrive(leftMotors, rightMotors)
  private val gyro = AHRS()

  private val speedMultiplier = AdjustableSpeed("Drivetrain Speed Multiplier", 0.8, Pos2D(0, 0))

  init {
    frontLeftMotor.setNeutralMode(NeutralMode.Brake)
    frontRightMotor.setNeutralMode(NeutralMode.Brake)
    backLeftMotor.setNeutralMode(NeutralMode.Brake)
    backRightMotor.setNeutralMode(NeutralMode.Brake)
    leftMotors.inverted = true

    leftEncoder.distancePerPulse = Drive.DISTANCE_PER_PULSE
    rightEncoder.distancePerPulse = Drive.DISTANCE_PER_PULSE

    Tabs.DIO.apply {
      add("Left Encoder (Meters)", leftEncoder).withPosition(0, 0).withSize(5, 4)
      add("Right Encoder (Meters)", rightEncoder).withPosition(5, 0).withSize(5, 4)
      add("Gyroscope", gyro).withPosition(10, 0).withSize(4, 4)
      add("Reset Left Encoder", InstantCommand({ leftEncoder.reset() }, this@DriveSubsystem)).withPosition(0, 4).withSize(5, 1)
      add("Reset Right Encoder", InstantCommand({ rightEncoder.reset() }, this@DriveSubsystem)).withPosition(5, 4).withSize(5, 1)
      add("Reset Gyroscope", InstantCommand({ gyro.reset() }, this@DriveSubsystem)).withPosition(10, 4).withSize(4, 1)
    }
  }

  override fun periodic() {
val result = RobotContainer.camera.latestResult
    if (!result.hasTargets()) return
    val bestTarget = result.bestTarget
//    val range = PhotonUtils.calculateDistanceToTargetMeters(
//      Drive.CAMERA_HEIGHT_METERS,
//      Drive.TARGET_HEIGHT_METERS,
//      Drive.CAMERA_PITCH_RADIANS,
//      Units.degreesToRadians(bestTarget.pitch)
//    )
    val range = bestTarget.bestCameraToTarget.x
    println("range = ${range}")
  }

  fun tankDrive(leftSpeed: Double, rightSpeed: Double) = drive.tankDrive(speedMultiplier.get() * leftSpeed, speedMultiplier.get() * rightSpeed)

  fun arcadeDrive(speed: Double, angularSpeed: Double) = drive.arcadeDrive(speed, angularSpeed)

  @Suppress("UsePropertyAccessSyntax")
  fun initTeleop(controller: CommandXboxController) = setDefaultCommand(TankDriveCommand(controller))

  fun stop() = drive.stopMotor()
}