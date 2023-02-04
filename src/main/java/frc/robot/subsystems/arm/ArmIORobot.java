package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class ArmIORobot implements ArmIO {

    private CANSparkMax arm;
    
    private RelativeEncoder armEncoder;

    public ArmIORobot(int armMotorID) {
        arm = new CANSparkMax(armMotorID, MotorType.kBrushless);

        arm.restoreFactoryDefaults(); 

        // Initializes AlternateEncoders from motors
        armEncoder = arm.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        // Sets position and velocity conversion factors so units are in degrees and degrees/second
        armEncoder.setPositionConversionFactor(360);
        armEncoder.setVelocityConversionFactor(60);
    }

    @Override
    public void updateInputs() {}

    @Override
    public double getOutputVoltage() {
        return MathUtil.clamp(arm.getOutputCurrent(), -12, 12);
    }

    @Override
    public double getCurrentAngleDegrees() {
        return armEncoder.getPosition();
    }

    @Override
    public double getVelocityDegreesPerSecond() {
        return armEncoder.getVelocity();
    }

    @Override
    public void setVoltage(double volts) {
        arm.setVoltage(volts);
    }
}
