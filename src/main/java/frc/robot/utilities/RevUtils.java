package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autos.AutoCommandManager;

// TODO Move used code to swerve module
public final class RevUtils {
  public static final double kFFDrive = AutoCommandManager.usePIDValueOrTune("kDriveFF", 0.15751);
  public static final double kPDrive = AutoCommandManager.usePIDValueOrTune("kDriveP", 0.23983);
  public static final double kIDrive = AutoCommandManager.usePIDValueOrTune("kDriveI", 0.0);
  public static final double kDDrive = AutoCommandManager.usePIDValueOrTune("kDriveD", 0.0);

  public static final double kFFTurn = AutoCommandManager.usePIDValueOrTune("kTurnFF", 0.0);
  public static final double kPTurn = AutoCommandManager.usePIDValueOrTune("kTurnP", 1.15);
  public static final double kITurn = AutoCommandManager.usePIDValueOrTune("kTurnI", 0.0);
  public static final double kDTurn = AutoCommandManager.usePIDValueOrTune("kTurnD", 0.1);

  public static void setTurnMotorConfig(CANSparkMax motorController) {
    // TODO Tune Manually
    motorController.getPIDController().setFF(kFFTurn);
    motorController.getPIDController().setP(kDTurn);
    motorController.getPIDController().setI(kITurn);
    motorController.getPIDController().setD(kDTurn);

    // TODO Help reduce CAN utilization
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

    motorController.setSmartCurrentLimit(40, 25);
  }

  public static void setDriveMotorConfig(CANSparkMax motorController) {
    // TODO TUNE
    motorController.getPIDController().setFF(kFFDrive);
    motorController.getPIDController().setP(kPDrive);
    motorController.getPIDController().setI(kIDrive);
    motorController.getPIDController().setD(kDDrive);

    // TODO Help reduce CAN utilization
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);

    motorController.setSmartCurrentLimit(60, 35);

    motorController.setOpenLoopRampRate(0.25);
    motorController.setOpenLoopRampRate(0.1);
  }

  public static SwerveModuleState optimize(
          SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
            placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
  
  public static void checkNeoError(REVLibError error, String message) {
    if (RobotBase.isReal() && error != REVLibError.kOk) {
        DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
    }
}

}