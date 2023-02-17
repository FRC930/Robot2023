package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

// TODO Move used code to swerve module
public final class RevUtils {
  public static void setTurnMotorConfig(CANSparkMax motorController) {
    // TODO Tune Manually
    motorController.getPIDController().setFF(0.0);
    motorController.getPIDController().setP(0.032);
    motorController.getPIDController().setI(0.0);
    // motorController.getPIDController().setD(0.02);

    // TODO Help reduce CAN utilization
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

    motorController.setSmartCurrentLimit(40, 25);
  }

  public static void setDriveMotorConfig(CANSparkMax motorController) {
    // TODO TUNE
    motorController.getPIDController().setFF(0.0);
    motorController.getPIDController().setP(0.1);
    motorController.getPIDController().setI(0.0);
    motorController.getPIDController().setD(0.0);

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