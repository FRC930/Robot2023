// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.SwerveModuleConstants;


public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules

  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftDriveEncoderPorts,
          DriveConstants.kFrontLeftTurningEncoderPorts,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftDriveEncoderPorts,
          DriveConstants.kRearLeftTurningEncoderPorts,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightDriveEncoderPorts,
          DriveConstants.kFrontRightTurningEncoderPorts,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightDriveEncoderPorts,
          DriveConstants.kRearRightTurningEncoderPorts,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

    public static final double kMaxSpeedMetersPerSecond = 3;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between front and back wheels on robot


    public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;
    
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public SwerveModulePosition[] mSwerveModPositions;

    public SwerveModule swerveModule1;
    public SwerveModule swerveModule2;
    public SwerveModule swerveModule3;
    public SwerveModule swerveModule4;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(SwerveModuleConstants frontLeftModuleConstants, SwerveModuleConstants frontRightModuleConstants, SwerveModuleConstants backLeftModuleConstants, SwerveModuleConstants backRightModuleConstants) {

    swerveOdometry = new SwerveDriveOdometry(swerveKinematics, getYaw(), mSwerveModPositions);

        swerveModule1 = new SwerveModule(0, frontLeftModuleConstants);
        swerveModule2 = new SwerveModule(1, frontRightModuleConstants);
        swerveModule3 = new SwerveModule(2, backLeftModuleConstants);
        swerveModule4 = new SwerveModule(3, backRightModuleConstants);

        mSwerveMods = new SwerveModule[] {  
            swerveModule1,
            swerveModule2,
            swerveModule3,
            swerveModule4
        };

        mSwerveModPositions = new SwerveModulePosition[] {
            swerveModule1.getPosition(),
            swerveModule2.getPosition(),
            swerveModule3.getPosition(),
            swerveModule4.getPosition()
        };
    }
  


  
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 0;
    public static final int kRearLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 1;
    public static final int kRearLeftTurningMotorPort = 3;
    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kRearRightTurningMotorPort = 7;

    public static final int[] kFrontLeftTurningEncoderPorts = new int[] {0, 1};
    public static final int[] kRearLeftTurningEncoderPorts = new int[] {2, 3};
    public static final int[] kFrontRightTurningEncoderPorts = new int[] {4, 5};
    public static final int[] kRearRightTurningEncoderPorts = new int[] {6, 7};

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final int[] kFrontLeftDriveEncoderPorts = new int[] {8, 9};
    public static final int[] kRearLeftDriveEncoderPorts = new int[] {10, 11};
    public static final int[] kFrontRightDriveEncoderPorts = new int[] {12, 13};
    public static final int[] kRearRightDriveEncoderPorts = new int[] {14, 15};

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;
  }
  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }
}
