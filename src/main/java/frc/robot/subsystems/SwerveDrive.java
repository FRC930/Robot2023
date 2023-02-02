// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.OdometryUtility;
import frc.robot.utilities.SwerveModuleConstants;



public class SwerveDrive extends SubsystemBase {
    public static final double kTrackWidth = Units.inchesToMeters(18.5);//0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);//0.7;
    // Distance between front and back wheels on robot

    public static final Translation2d[] kModuleTranslations = {
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };

    public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(kModuleTranslations);


    private Pigeon2 m_pigeon = new Pigeon2(13, "rio"); //TODO pass in id and canbus   CAN.pigeon);

    private SwerveDriveOdometry m_odometry;
    
    // private AprilVisionUtility m_aprilCameraOne;
    private OdometryUtility m_aprilCameraOne;

    private double m_simYaw;
    //TODO TUNE FOR GHOST
    public static final double kPXController = 10.18; //0.076301;
    public static final double kPYController = 7.596; //0.076301;

    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(14.5);// 3;
    //TODO VALIDATE AND TURN
    private static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0; // Last year 11.5?
    private static final boolean invertGyro = false;

    //TODO they use ProfiledPIDController
    private PIDController autoXController;
    private PIDController autoYController;
    private PIDController autoThetaController;
    private PIDController autoPitchController;

    private SwerveModuleState[] moduleStates; 

    private SwerveModule[] mSwerveMods;

    public SwerveDrive(SwerveModuleConstants frontLeftModuleConstants, SwerveModuleConstants frontRightModuleConstants, SwerveModuleConstants backLeftModuleConstants, SwerveModuleConstants backRightModuleConstants) {

      mSwerveMods = new SwerveModule[] {  
        new SwerveModule(0, frontLeftModuleConstants),
        new SwerveModule(1, frontRightModuleConstants),
        new SwerveModule(2, backLeftModuleConstants),
        new SwerveModule(3, backRightModuleConstants)
      };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
        * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
        */
        Timer.delay(1.0);
        resetAngleToAbsolute();
        
        m_odometry = new SwerveDriveOdometry(
                kDriveKinematics,
                getHeadingRotation2d(),
                getModulePositions(),
                new Pose2d());
    
        m_pigeon.setYaw(0);
        //used for Autonous
        autoXController = new PIDController(kPXController, 0, 0);
        autoYController = new PIDController(kPYController, 0, 0);
        autoThetaController = new PIDController(
        1.33, 0, 0);
        autoPitchController = new PIDController(1, 0, 0);
        
    // m_aprilCameraOne = new AprilVisionUtility(kDriveKinematics, getHeadingRotation2d(), getModulePositions(), getPoseMeters());
    m_aprilCameraOne = new OdometryUtility(kDriveKinematics, getHeadingRotation2d(), getModulePositions(), getPoseMeters());
  }

  public Pose2d getPose(){
    return m_aprilCameraOne.getPose();
  }

  public void drive(
          double throttle,
          double strafe,
          double rotation,
          boolean isFieldRelative,
          boolean isOpenLoop){
    
    //Applies a Deadband of 0.05 to the controllers input
    throttle = throttle * kMaxSpeedMetersPerSecond;
    strafe = strafe * kMaxSpeedMetersPerSecond;
    rotation = rotation * kMaxRotationRadiansPerSecond;

    ChassisSpeeds chassisSpeeds =
            isFieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    throttle, strafe, rotation, getHeadingRotation2d())
                    : new ChassisSpeeds(throttle, strafe, rotation);

    moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setSwerveModuleStates(moduleStates, isOpenLoop);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

    for (int i = 0; i < mSwerveMods.length; i++) {
        SwerveModule module = mSwerveMods[i];
        module.setDesiredState(states[i], isOpenLoop);
    }
  }

  public double getHeadingDegrees() {
    return Math.IEEEremainder(m_pigeon.getYaw(), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return getYaw();
    // return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  

  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }


  public SwerveModule getSwerveModule(int moduleNumber) {
    return mSwerveMods[moduleNumber];
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        mSwerveMods[0].getState(),
        mSwerveMods[1].getState(),
        mSwerveMods[2].getState(),
        mSwerveMods[3].getState()
    };
  }
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        mSwerveMods[0].getPosition(),
        mSwerveMods[1].getPosition(),
        mSwerveMods[2].getPosition(),
        mSwerveMods[3].getPosition()
    };
  }

  public void updateOdometry() {
    m_odometry.update(getHeadingRotation2d(), getModulePositions());

    for (int i = 0; i < mSwerveMods.length; i++) {
      SwerveModule module = mSwerveMods[i];
      var modulePositionFromChassis =
              kModuleTranslations[i]
                      .rotateBy(getHeadingRotation2d())
                      .plus(getPoseMeters().getTranslation());
//TODO WHAT IS THIS FOR 
      module.setModulePose(
              new Pose2d(
                      modulePositionFromChassis,
                      module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
    //Logs information about the robot with AdvantageScope
    Logger.getInstance().recordOutput("SwerveModuleStates/Measured",
        getModuleStates());

    // Log odometry pose
    Logger.getInstance().recordOutput("Odometry/Robot", m_odometry.getPoseMeters());


  }

  private void updateSmartDashboard() {}

  @Override
  public void periodic() {
    updateOdometry();
    updateSmartDashboard();
    
    //Logs information about the robot with AdvantageScope
    Logger.getInstance().recordOutput("Drive/Gryo" + "connected", m_pigeon.getLastError().equals(ErrorCode.OK));
    Logger.getInstance().recordOutput("Drive/Gryo" + "positionUnits", m_pigeon.getYaw());
    double[] xyzDps = new double[3];
    m_pigeon.getRawGyro(xyzDps);
    Logger.getInstance().recordOutput("Drive/Gyro" + "velocityRadPerSec", Units.degreesToRadians(xyzDps[2]));
    if (moduleStates != null) {
      Logger.getInstance().recordOutput("SwerveModuleStates/Subsystem", moduleStates);
    }
    m_aprilCameraOne.updateCameraPos(getHeadingRotation2d(), getModulePositions(), getPoseMeters());
    //kDriveKinematics, getHeadingRotation2d(), getModulePositions(), getPoseMeters()
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed = kDriveKinematics.toChassisSpeeds(getModuleStates());
    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(20);
    m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
  }

    public static SwerveDriveKinematics getSwerveKinematics() {
        return kDriveKinematics;
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
    
    public PIDController getAutoPitchController() {
      return autoPitchController;
    }
    public void resetOdometry(Pose2d initialPose) {
        m_odometry.resetPosition(getYaw(), getModulePositions(), initialPose);
    }
     


    private Rotation2d getYaw() {
      double[] ypr = new double[3];
      m_pigeon.getYawPitchRoll(ypr);
      return (invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
   }

    public Rotation2d getPitch() {
      double[] ypr = new double[3];
      m_pigeon.getYawPitchRoll(ypr);
      return (invertGyro) ? Rotation2d.fromDegrees(360 - ypr[1]) : Rotation2d.fromDegrees(ypr[1]);
    }

    public void setSwerveModuleStates(SwerveModuleState[] states) {
        setSwerveModuleStates(states, false);
    }
  /**
   * Resets each SwerveModule to the absolute position.
   */
  public void resetAngleToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetAngleToAbsolute();
    }
  }
}