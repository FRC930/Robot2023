// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

import org.littletonrobotics.junction.Logger;


import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.CtreUtils;
import frc.robot.utilities.RevUtils;
import frc.robot.utilities.SwerveModuleConstants;


public class SwerveModule extends SubsystemBase {
  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 0;// Auto/ closed loop requires this to be 0 and not 1
  private final int SIM_SLOT = 2;

  //verified Values
  //https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777303153
  //MK4I L2
  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(14.5);
  public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
  public static final double kMaxModuleAngularSpeedRadiansPerSecond = 180; //2 * Math.PI; last year 930 used 180
  public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 180; //2 * Math.PI; last year 930 used 180
  public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12; //TODO
  public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12; //TODO
  public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12; //TODO
  public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5 //TODO REMOVE
  public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3 //TODO REMOVEE
  //Verified Values
  // https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module?variant=39598777303153
  public static final double kDriveMotorGearRatio = 6.75; // MK4i L2 Neo
  public static final double kTurningMotorGearRatio = 150.0/7.0; // MK4i turning ratio MK4i Neo
  public static final int kNeoCPR = 42;
  public static final int kNeoRPM = 5676;
  public static final double kDriveRevToMeters =
            ((kWheelDiameterMeters * Math.PI) / kDriveMotorGearRatio);
  public static final double kDriveRpmToMetersPerSecond =
            kDriveRevToMeters/ 60.0;
  public static final double kTurnRotationsToDegrees =
            360.0 / kTurningMotorGearRatio;

  // TODO use LoggedTunableNumber for PID values see advantagekit example

  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;
  private CANCoder m_angleEncoder;

  public final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;
  
 
  private double m_angleOffset;

  private final SparkMaxPIDController m_driveController;
  private SparkMaxPIDController m_turnController;

  double m_currentAngle;
  double m_lastAngle;

  private double m_simDriveEncoderPosition;
  private double m_simDriveEncoderVelocity;
  private double m_simAngleDifference;
  private double m_simTurnAngleIncrement;
  private int m_moduleNumber;
  private Pose2d m_pose;

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
    m_driveMotor = new CANSparkMax(swerveModuleConstants.driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(swerveModuleConstants.turningMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_angleEncoder = new CANCoder(swerveModuleConstants.cancoderID, "rio"); // TODO CanBUS Pass in
    m_angleOffset = swerveModuleConstants.angleOffset;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveMotor.restoreFactoryDefaults();
    RevUtils.setDriveMotorConfig(m_driveMotor);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_driveMotor.setInverted(true);//MK4i drive motor is inverted

    m_turningMotor.restoreFactoryDefaults();
    RevUtils.setTurnMotorConfig(m_turningMotor);
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_turningMotor.setSmartCurrentLimit(25);
    m_turningMotor.enableVoltageCompensation(12.6);
    m_turningMotor.setInverted(true); // MK4i Steer Motor is inverted

    m_angleEncoder.configFactoryDefault();
    CtreUtils.checkCtreError(
    m_angleEncoder.configAllSettings(CtreUtils.generateCanCoderConfig()),
      "Error setting configuration CANCoder encoder");

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(kDriveRevToMeters);
    m_driveEncoder.setVelocityConversionFactor(kDriveRpmToMetersPerSecond);
    m_driveEncoder.setPosition(0);

    m_turnEncoder = m_turningMotor.getEncoder();
    RevUtils.checkNeoError(
    m_turnEncoder.setPositionConversionFactor(kTurnRotationsToDegrees),
      "Error setting Position Converstion on turn motor");
    RevUtils.checkNeoError(  
    m_turnEncoder.setVelocityConversionFactor(kTurnRotationsToDegrees / 60),
      "Error setting Velocity Converstion on turn motor");

    // REVs
    m_driveController = m_driveMotor.getPIDController();
    // NOTE: TODO look at if m_driveController.set... if using LoggedTunableNumber        
    m_turnController = m_turningMotor.getPIDController();

    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_turningMotor, DCMotor.getNEO(1));
      m_driveController.setP(1, SIM_SLOT);
    }

    //reset angle to Absolute is being called from swerve drive
  }

  // TODO DO we need? maybe for debugging in
  public SwerveModule(int moduleNumber, SwerveModuleConstants frontLeftModuleConstants) {
    this(frontLeftModuleConstants);
    m_moduleNumber = moduleNumber;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
  }

  public void resetAngleToAbsolute() {
    double angle = m_angleEncoder.getAbsolutePosition() - m_angleOffset;
    m_turnEncoder.setPosition(angle);
  }

  public double getHeadingDegrees() {
    if(RobotBase.isReal())
      return m_turnEncoder.getPosition();
    else
      return m_currentAngle;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getDriveMeters() {
    if(RobotBase.isReal())
      return m_driveEncoder.getPosition();
    else
      return m_simDriveEncoderPosition;
  }
  public double getDriveMetersPerSecond() {
    if(RobotBase.isReal())
      return m_driveEncoder.getVelocity();
    else
      return m_simDriveEncoderVelocity;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    //TODO re-enable optimize once swerve module is working
    desiredState = RevUtils.optimize(desiredState, getHeadingRotation2d());

    //Logs information about the robot with AdvantageScope
    double velocityRadPerSec = 
    desiredState.speedMetersPerSecond / ((kWheelDiameterMeters/2) * kDriveMotorGearRatio * (2 * Math.PI));
  Logger.getInstance().recordOutput(
    "SwerveSetPointValue/DriveRadSec/" + Integer.toString(getModuleNumber()),
    velocityRadPerSec);
  Logger.getInstance().recordOutput(
    "SwerveSetPointValue/DriveRadMin/" + Integer.toString(getModuleNumber()),
    velocityRadPerSec * 60.0);
  Logger.getInstance().recordOutput(
    "SwerveSetPointValue/DriveM/" + Integer.toString(getModuleNumber()),
    desiredState.speedMetersPerSecond);

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeedMetersPerSecond;
      m_driveMotor.set(percentOutput);
    } else {
      //TODO currently doesn't work
      int DRIVE_PID_SLOT = RobotBase.isReal() ? VEL_SLOT : SIM_SLOT;
      m_driveController.setReference(
              desiredState.speedMetersPerSecond,
              CANSparkMax.ControlType.kVelocity,
              DRIVE_PID_SLOT
      );
    }

    double angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxSpeedMetersPerSecond * 0.01))
                    ? m_lastAngle
                    : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents Jittering.
    angle = desiredState.angle.getDegrees();
    m_turnController.setReference(angle, CANSparkMax.ControlType.kPosition, POS_SLOT);
    // TODO REVExample was missing this line (m_angle did not make sense otherwise)!!!
    m_lastAngle = angle;
    Logger.getInstance().recordOutput(
      "SwerveSetPointValue/TurnD/" + Integer.toString(getModuleNumber()),
    (angle));
    Logger.getInstance().recordOutput(
      "SwerveSetPointValue/TurnDC/" + Integer.toString(getModuleNumber()),
    (m_angleEncoder.getAbsolutePosition()));

    //Logs information about the robot with AdvantageScope
    Logger.getInstance().recordOutput(
      "SwerveSetPointValue/Turn/" + Integer.toString(getModuleNumber()),
    Units.degreesToRadians(angle));

    if (RobotBase.isSimulation()) {
      simUpdateDrivePosition(desiredState);
//      simTurnPosition(angle); // TODO Determine why commented out in REVexample
      m_currentAngle = angle;

    }
  }

  //gets the swerve module number
  private int getModuleNumber() {
    return m_moduleNumber;
  }

  private void simUpdateDrivePosition(SwerveModuleState state) {
    m_simDriveEncoderVelocity = state.speedMetersPerSecond;
    double distancePer20Ms = m_simDriveEncoderVelocity / 50.0;

    m_simDriveEncoderPosition += distancePer20Ms;
  }
  private void simTurnPosition(double angle) {
    if (angle != m_currentAngle && m_simTurnAngleIncrement == 0) {
      m_simAngleDifference = angle - m_currentAngle;
      m_simTurnAngleIncrement = m_simAngleDifference / 20.0;// 10*20ms = .2 sec move time
    }

    if (m_simTurnAngleIncrement != 0) {
      m_currentAngle += m_simTurnAngleIncrement;

      if ((Math.abs(angle - m_currentAngle)) < .1) {
        m_currentAngle = angle;
        m_simTurnAngleIncrement = 0;
      }
    }
  }


  public SwerveDriveKinematics getSwerveKinematics() {
    return SwerveDrive.kDriveKinematics;
  }


  // TODO NOT SURE WHAT FOR m_pose never gotten
  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }
  // TODO NOT SURE WHAT FOR
  public Pose2d getModulePose() {
    return m_pose;
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
