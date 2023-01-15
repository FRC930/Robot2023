// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SwerveModuleConstants;


import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
        public static final double kTrackWidth = Units.inchesToMeters(24.0);//0.5;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.0);//0.7;
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

  private SwerveDriveOdometry m_odometry;// TODO remove need to do in constructor =
        //   new SwerveDriveOdometry(
        //           kDriveKinematics,
        //           getHeadingRotation2d(),
        //           getModulePositions(),
        //           new Pose2d());
// TODO what needed for 
//   private ProfiledPIDController m_xController =
//           new ProfiledPIDController(kP_X, 0, kD_X, kThetaControllerConstraints);
//   private ProfiledPIDController m_yController =
//           new ProfiledPIDController(kP_Y, 0, kD_Y, kThetaControllerConstraints);
//   private ProfiledPIDController m_turnController =
//           new ProfiledPIDController(kP_Theta, 0, kD_Theta, kThetaControllerConstraints);

  private double m_simYaw;

  public static final double kPXController = 0.4; //0.076301;
  public static final double kPYController = 0.4; //0.076301;
  
public static final double kMaxSpeedMetersPerSecond = 3;
private static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;

  //TODO they use ProfiledPIDController
  public PIDController autoXController;
  public PIDController autoYController;
  public PIDController autoThetaController;

  
  private SwerveModule swerveModule1;
  private SwerveModule swerveModule2;
  private SwerveModule swerveModule3;
  private SwerveModule swerveModule4;
private SwerveModule[] mSwerveMods;
private SwerveModulePosition[] mSwerveModPositions;

  public SwerveDrive(SwerveModuleConstants frontLeftModuleConstants, SwerveModuleConstants frontRightModuleConstants, SwerveModuleConstants backLeftModuleConstants, SwerveModuleConstants backRightModuleConstants) {

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
    
        m_odometry = new SwerveDriveOdometry(
                kDriveKinematics,
                getHeadingRotation2d(),
                getModulePositions(),
                new Pose2d());
    
    m_pigeon.setYaw(0);
    // TODO WHAT ARE THESE for
//     autoXController = new PIDController(kPXController, 0, 0);
//     autoYController = new PIDController(kPYController, 0, 0);
//     autoThetaController = new PIDController(
//         0.33, 0, 0);
  }

  public void drive(
          double throttle,
          double strafe,
          double rotation,
          boolean isFieldRelative,
          boolean isOpenLoop) {
    throttle *= kMaxSpeedMetersPerSecond;
    strafe *= kMaxSpeedMetersPerSecond;
    rotation *= kMaxRotationRadiansPerSecond;

    ChassisSpeeds chassisSpeeds =
            isFieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    throttle, strafe, rotation, getHeadingRotation2d())
                    : new ChassisSpeeds(throttle, strafe, rotation);

    SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeedMetersPerSecond);

    for (int i = 0; i < mSwerveMods.length; i++) {
        SwerveModule module = mSwerveMods[i];
        module.setDesiredState(moduleStates[i], isOpenLoop);
    }
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
    return Rotation2d.fromDegrees(getHeadingDegrees());
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
  }

  private void updateSmartDashboard() {}

  @Override
  public void periodic() {
    updateOdometry();
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed = kDriveKinematics.toChassisSpeeds(getModuleStates());
    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(20);
    m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));

        // Log odometry pose
   Logger.getInstance().recordOutput("Odometry/Robot", m_odometry.getPoseMeters());

  }

    public static SwerveDriveKinematics getSwerveKinematics() {
        return kDriveKinematics;
    }
//     public PIDController getAutoXController() {
//         return autoXController;
//     }

//     public PIDController getAutoYController() {
//         return autoYController;
//     }

//     public PIDController getAutoThetaController() {
//         return autoThetaController;
//     }

    public void resetOdometry(Pose2d initialPose) {
        m_odometry.resetPosition(getYaw(), getModulePositions(), getPoseMeters());
    }

    private Rotation2d getYaw() {
        // TODO gyro?
        return getPoseMeters().getRotation();
   }

    public void setSwerveModuleStates(SwerveModuleState[] states) {
        setSwerveModuleStates(states, false);
    }

}