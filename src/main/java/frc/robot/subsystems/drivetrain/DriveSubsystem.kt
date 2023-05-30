package frc.robot.subsystems.drivetrain

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.RamseteAutoBuilder
import com.pathplanner.lib.commands.PPRamseteCommand
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Drive
import frc.robot.Tabs
import frc.robot.shuffleboard.AdjustableSpeed
import frc.robot.shuffleboard.Pos2D

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
  @JvmStatic private val KINEMATICS = DifferentialDriveKinematics(Drive.TRACK_WIDTH_METERS)
  private val poseEstimator = DifferentialDrivePoseEstimator(
    KINEMATICS,
    gyro.rotation2d,
    0.0,
    0.0,
    Pose2d(),
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5.0)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30.0))
  )

  private val speedMultiplier = AdjustableSpeed("Drivetrain Speed Multiplier", 0.8, Pos2D(0, 0))

  init {
    frontLeftMotor.setNeutralMode(NeutralMode.Brake)
    frontRightMotor.setNeutralMode(NeutralMode.Brake)
    backLeftMotor.setNeutralMode(NeutralMode.Brake)
    backRightMotor.setNeutralMode(NeutralMode.Brake)
    leftMotors.inverted = true

    // TODO: One of these encoders need to be inverted lol
    leftEncoder.distancePerPulse = Drive.DISTANCE_PER_PULSE
    rightEncoder.distancePerPulse = Drive.DISTANCE_PER_PULSE

    drive.isSafetyEnabled = false

    Tabs.DIO.apply {
      add("Left Encoder (Meters)", leftEncoder).withPosition(0, 0).withSize(5, 4)
      add("Right Encoder (Meters)", rightEncoder).withPosition(5, 0).withSize(5, 4)
      add("Gyroscope", gyro).withPosition(10, 0).withSize(4, 4)
      add("Reset Left Encoder", InstantCommand((leftEncoder::reset), this@DriveSubsystem)).withPosition(0, 4).withSize(5, 1)
      add("Reset Right Encoder", InstantCommand((rightEncoder::reset), this@DriveSubsystem)).withPosition(5, 4).withSize(5, 1)
      add("Reset Gyroscope", InstantCommand((gyro::reset), this@DriveSubsystem)).withPosition(10, 4).withSize(4, 1)
    }

    resetEncoders()
    resetGyro()
  }

  override fun periodic() {
    poseEstimator.update(getGyroRotation2d(), getLeftDistance(), getRightDistance())

    // TODO: Steal https://github.com/STMARobotics/frc-7028-2023/blob/main/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java#LL114C28-L114C28
  }

  /**
   * Uses current pose to get to a new pose
   * @param endingRelativePose the [Translation2d] is used and [Rotation2d] is used relative to the current pose
   */
  fun loadRelativeTrajectoryToRamseteCommand(constraints: PathConstraints, endingRelativePose: Pose2d): SequentialCommandGroup {
    val translation = getEstimatedPose().translation
    val rotation = getEstimatedPose().rotation
    val trajectory = PathPlanner.generatePath(
      constraints,
      PathPoint(translation, rotation),
      PathPoint(translation.plus(endingRelativePose.translation), rotation.plus(endingRelativePose.rotation))
    )
    return PPRamseteCommand(
      trajectory,
      ::getEstimatedPose,
      RamseteController(Drive.RAMSETE_B, Drive.RAMSETE_ZETA),
      SimpleMotorFeedforward(Drive.KS_VOLTS, Drive.KV_VOLT_SECONDS_PER_METER, Drive.KA_VOLT_SECONDS_SQUARED_PER_METER),
      getKinematics(),
      ::getWheelSpeeds,
      PIDController(Drive.P_DRIVE_VELOCITY, 0.0, 0.0),
      PIDController(Drive.P_DRIVE_VELOCITY, 0.0, 0.0),
      ::tankDriveVolts,
      true,
      this
    ).andThen(InstantCommand(::stop))
  }

  /**
   * For autonomous modes
   */
  fun loadPathPlannerTrajectoryToRamseteCommand(fileName: String, constraints: PathConstraints, reversed: Boolean=false, eventMap: (Map<String, Command>)=emptyMap()): SequentialCommandGroup {
    val path = PathPlanner.loadPathGroup(fileName, reversed, constraints)
    val autoBuilder = RamseteAutoBuilder(
      ::getEstimatedPose,
      ::resetOdometry,
      RamseteController(Drive.RAMSETE_B, Drive.RAMSETE_ZETA),
      getKinematics(),
      SimpleMotorFeedforward(Drive.KS_VOLTS, Drive.KV_VOLT_SECONDS_PER_METER, Drive.KA_VOLT_SECONDS_SQUARED_PER_METER),
      ::getWheelSpeeds,
      PIDConstants(Drive.P_DRIVE_VELOCITY, 0.0, 0.0),
      ::tankDriveVolts,
      eventMap,
      true,
      this
    )

    return autoBuilder.fullAuto(path).andThen(InstantCommand(::stop))
  }

  fun tankDrive(leftSpeed: Double, rightSpeed: Double) = drive.tankDrive(speedMultiplier.get() * leftSpeed, speedMultiplier.get() * rightSpeed)

  fun arcadeDrive(speed: Double, angularSpeed: Double) = drive.arcadeDrive(speed, angularSpeed)

  private fun tankDriveVolts(leftVolts: Double, rightVolts: Double) {
    leftMotors.setVoltage(-leftVolts)
    rightMotors.setVoltage(-rightVolts)
    drive.feed()
  }

  @Suppress("UsePropertyAccessSyntax")
  fun initTeleop(controller: CommandXboxController) = setDefaultCommand(TankDriveCommand(controller))

  fun stop() = drive.stopMotor()

  fun getEstimatedPose() = poseEstimator.estimatedPosition

  fun getGyroAngle() = gyro.angle

  fun getGyroRotation2d() = gyro.rotation2d

  fun getLeftDistance() = leftEncoder.distance

  fun getRightDistance() = rightEncoder.distance

  fun getAverageDistance() = (getLeftDistance() + getRightDistance()) / 2.0

  fun getKinematics() = KINEMATICS

  fun resetOdometry(pose: Pose2d) {
    resetEncoders()
    resetGyro()
    poseEstimator.resetPosition(gyro.rotation2d, 0.0, 0.0, pose)
  }

  fun resetEncoders() {
    leftEncoder.reset()
    rightEncoder.reset()
  }

  fun resetGyro() = gyro.reset()

  fun getWheelSpeeds() = DifferentialDriveWheelSpeeds(leftEncoder.rate, rightEncoder.rate)
}