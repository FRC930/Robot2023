package frc.robot.utilities;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

// TODO Move used code to swerve module
public final class CtreUtils {
  public static TalonFXConfiguration generateTurnMotorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.slot0.kF = 0.0;
    motorConfig.slot0.kP = 0.6;
    motorConfig.slot0.kI = 0.0;
    motorConfig.slot0.kD = 12.0;

    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
    motorConfig.supplyCurrLimit = supplyCurrentLimit;

    motorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    return motorConfig;
  }

  public static TalonFXConfiguration generateDriveMotorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.slot0.kF = 0.0;
    motorConfig.slot0.kP = 0.1;
    motorConfig.slot0.kI = 0.0;
    motorConfig.slot0.kD = 0.0;

    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
    motorConfig.supplyCurrLimit = supplyCurrentLimit;

    motorConfig.openloopRamp = 0.25;
    motorConfig.closedloopRamp = 0.1;

    motorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    return motorConfig;
  }

  /**
   * <h3>generateCanCoderConfig</h3>
   * 
   * Generates and returns the configuration of the sensor.
   * @return - The config of the sensor
   */
  public static CANCoderConfiguration generateCanCoderConfig() {
    CANCoderConfiguration sensorConfig = new CANCoderConfiguration();

    sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    // https://github.com/FRC5892/ChargedUp2023/commit/ad06f53ce54cbc78df013fd333e2b98b9cc0454c
    // sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    // // But may want optimize back on
    sensorConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    sensorConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    return sensorConfig;
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
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
   * @param newAngle       Target Angle
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

  /**
   * <h3>checkCtreError</h3>
   * 
   * Checks for a specified error.
   * @param errorCode - Code of error
   * @param message - Desired message if error detected
   */
  public static void checkCtreError(ErrorCode errorCode, String message) {
    if (RobotBase.isReal() && errorCode != ErrorCode.OK) {
      DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
    }
  }
}