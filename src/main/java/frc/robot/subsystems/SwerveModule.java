// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Utilities.SwerveModuleConstants;


public class SwerveModule {

  public static final double kWheelDiameterMeters = 0.15;
  public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
  public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
  public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
  public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
  public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;
  public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
  public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
  public static final double kDriveMotorGearRatio = 6.12;
  public static final double kTurningMotorGearRatio = 12.8;
  public static final int kNeoCPR = 42;
  public static final int kCANCoderCPR = 4096; // Figure this out for Neo Motors.
  public static final double kDriveRevToMeters =
            ((kWheelDiameterMeters * Math.PI) / kDriveMotorGearRatio);
  public static final double kDriveRpmToMetersPerSecond =
            kDriveRevToMeters / 60.0;
  public static final double kTurnRotationsToDegrees =
            360.0 / kTurningMotorGearRatio;


  public static final double kPModuleTurningController = 1;
  public static final double kPModuleDriveController = 1;

  public static final boolean invertGyro = false;
  public static final double trackWidth = Units.inchesToMeters(23);
  public static final double wheelBase = Units.inchesToMeters(23);
  
  private final PIDController m_drivePIDController =
      new PIDController(kPModuleDriveController, 0, 0);

  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;
  private CANCoder m_moduleEncoder;
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ksDriveVoltSecondsPerMeter, kaDriveVoltSecondsSquaredPerMeter, kvDriveVoltSecondsSquaredPerMeter);
private final ProfiledPIDController m_turningProfiledPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI))

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kMaxModuleAngularSpeedRadiansPerSecond,
              kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
    new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
    new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
    new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
    new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));  
  
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public SwerveModulePosition[] mSwerveModPositions;
  public Pigeon2 gyro;
  public PIDController autoXController;
  public PIDController autoYController;
  public PIDController autoThetaController;
  public SwerveModule swerveModule1;
  public SwerveModule swerveModule2;
  public SwerveModule swerveModule3;
  public SwerveModule swerveModule4;
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(SwerveModuleConstants swerveModuleConstants) { 
    m_driveMotor = new CANSparkMax(swerveModuleConstants.driveEncoderChannel, CANSparkMaxLowLevel.MotorType.kBrushed);
    m_turningMotor = new CANSparkMax(swerveModuleConstants.turningEncoderChannel, CANSparkMaxLowLevel.MotorType.kBrushed);

    m_moduleEncoder = new CANCoder(swerveModuleConstants.cancoderID);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveMotor.restoreFactoryDefaults();
    setDriveMotorConfig(m_driveMotor);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_turningMotor.restoreFactoryDefaults();
    setTurnMotorConfig(m_turningMotor);
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_moduleEncoder.configFactoryDefault();
    m_moduleEncoder.configAllSettings(generateCanCoderConfig());

    m_driveMotor.getEncoder().setPositionConversionFactor(kDriveRevToMeters);
    m_driveMotor.getEncoder().setVelocityConversionFactor(kDriveRpmToMetersPerSecond);

    m_turningMotor.getEncoder().setPositionConversionFactor(kTurnRotationsToDegrees);
    m_turningMotor.getEncoder().setVelocityConversionFactor(kTurnRotationsToDegrees / 60);

    
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }

  public SwerveDriveKinematics getSwerveKinematics() {
    return swerveKinematics;
}

public PIDController getAutoXController() {
    return autoXController;
}

public PIDController getAutoYController() {
    return autoYController;
}

public PIDController getAutoThetaController() {
    return autoThetaController;
}

public void resetOdometry(Pose2d pose) {
  swerveOdometry.resetPosition(getYaw(), mSwerveModPositions, pose);
}

public Rotation2d getYaw() {
  double[] ypr = new double[3];
  gyro.getYawPitchRoll(ypr);
  return (invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
}
  private static void setDriveMotorConfig(CANSparkMax motorController) {
    motorController.getPIDController().setFF(0.0);
    motorController.getPIDController().setP(0.1);
    motorController.getPIDController().setI(0.0);
    motorController.getPIDController().setD(0.0);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);
    motorController.setSmartCurrentLimit(60, 35);
    motorController.setOpenLoopRampRate(0.25);
    motorController.setOpenLoopRampRate(0.1);
  }

  private static void setTurnMotorConfig(CANSparkMax motorController) {
    motorController.getPIDController().setFF(0.0);
    motorController.getPIDController().setP(0.2);
    motorController.getPIDController().setI(0.0);
    // motorController.getPIDController().setD(12.0);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    motorController.setSmartCurrentLimit(40, 25);
  }
  private static CANCoderConfiguration generateCanCoderConfig() {
    CANCoderConfiguration sensorConfig = new CANCoderConfiguration();
    sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    sensorConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    sensorConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    return sensorConfig;
  }
}
