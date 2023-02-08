package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmIORobot implements ArmIO {

    private CANSparkMax arm;
    
    private AbsoluteEncoder armEncoder;

    public ArmIORobot(int armMotorID) {
        arm = new CANSparkMax(armMotorID, MotorType.kBrushless);

        arm.restoreFactoryDefaults(); 

        // Initializes Absolute Encoders from motors
        armEncoder = arm.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        // Sets position and velocity conversion factors so units are in degrees and degrees/second
        armEncoder.setPositionConversionFactor(360);
        armEncoder.setVelocityConversionFactor(60);

        armEncoder.setInverted(true);
        arm.setInverted(true);
    }

    @Override
    public void updateInputs() {}

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
