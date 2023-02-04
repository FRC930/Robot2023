package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.MathUtil;

public class ManipulatorIORobot implements ManipulatorIO { 

    private CANSparkMax manipulator;

    private RelativeEncoder manipulatorEncoder;

    public ManipulatorIORobot(int manipulatorMotorID) {
        manipulator = new CANSparkMax(manipulatorMotorID, MotorType.kBrushless);

        manipulator.restoreFactoryDefaults();

        // Initializes AlternateEncoders from motors
        manipulatorEncoder = manipulator.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        // Sets position and velocity conversion factors so units are in degrees and degrees/second
        manipulatorEncoder.setPositionConversionFactor(360);
        manipulatorEncoder.setVelocityConversionFactor(60);
    }

    @Override
    public void updateInputs() {}

    @Override
    public double getOutputVoltage() {
        return MathUtil.clamp(manipulator.getOutputCurrent(), -12, 12);
    }

    @Override
    public double getCurrentAngleDegrees() {
        return manipulatorEncoder.getPosition();
    }

    @Override
    public double getVelocityDegreesPerSecond() {
        return manipulatorEncoder.getVelocity();
    }

    @Override
    public void setVoltage(double volts) {
        manipulator.setVoltage(volts);
        
    }
}
