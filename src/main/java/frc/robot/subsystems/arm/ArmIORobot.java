package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class ArmIORobot implements ArmIO {

    private CANSparkMax wristMotor;
    private CANSparkMax shoulderMotor; 

    private RelativeEncoder wristEncoder;
    private RelativeEncoder shoulderEncoder;

    public ArmIORobot(int WristMotorID, int ShoulderMotorID) {
        wristMotor = new CANSparkMax(WristMotorID, MotorType.kBrushless);
        shoulderMotor = new CANSparkMax(ShoulderMotorID, MotorType.kBrushless);

        wristMotor.restoreFactoryDefaults();
        shoulderMotor.restoreFactoryDefaults(); 

        // Initializes AlternateEncoders from motors
        wristEncoder = wristMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        shoulderEncoder = shoulderMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        // Sets position and velocity conversion factors so units are in degrees and degrees/second
        wristEncoder.setPositionConversionFactor(360);
        wristEncoder.setVelocityConversionFactor(60);
        shoulderEncoder.setPositionConversionFactor(360);
        shoulderEncoder.setVelocityConversionFactor(60);
    }

    @Override
    public void updateInputs() {}

    @Override
    public double getWristOutputVoltage() {
        return MathUtil.clamp(wristMotor.getOutputCurrent(), -12, 12);
    }

    @Override
    public double getWristCurrentAngleDegrees() {
        return wristEncoder.getPosition();
    }

    @Override
    public double getWristVelocityDegreesPerSecond() {
        return wristEncoder.getVelocity();
    }

    @Override
    public void setWristVoltage(double volts) {
        wristMotor.setVoltage(volts);
        
    }

    @Override
    public double getShoulderOutputVoltage() {
        return shoulderMotor.getOutputCurrent();
    }

    @Override
    public double getShoulderCurrentAngleDegrees() {
        return shoulderEncoder.getPosition();
    }

    @Override
    public double getShoulderVelocityDegreesPerSecond() {
        return shoulderEncoder.getVelocity();
    }

    @Override
    public void setShoulderVoltage(double volts) {
        shoulderMotor.setVoltage(volts);
    }
}
