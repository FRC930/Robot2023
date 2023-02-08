package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

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

    /**
     * <h3>updateInputs</h3>
     * 
     * Left blank because it's only used in simulation
     */
    @Override
    public void updateInputs() {}

    /**
     * <h3>getOutputVoltage</h3>
     * 
     * Gets the shoulder motor outputs in volts
     * @return the sholder motor output voltage
     */
    @Override
    public double getOutputVoltage() {
        return MathUtil.clamp(arm.getOutputCurrent(), -12, 12);
    }

    /**
     * <h3>getCurrentAngleDegrees</h3>
     * 
     * Gets the shoulder motor position in degrees
     * @return the shoulder motor position
     */
    @Override
    public double getCurrentAngleDegrees() {
        return armEncoder.getPosition();
    }

    /**
     * <h3>getVelocityDegreesPerSecond</h3>
     * 
     * Gets the shoulder motor's velocity
     * @return velocity of arm motor
     */
    @Override
    public double getVelocityDegreesPerSecond() {
        return armEncoder.getVelocity();
    }

    /**
     * <h3>setVoltage</h3>
     * 
     * Set the shoulder motor voltage 
     * @param volts
     */
    @Override
    public void setVoltage(double volts) {
        arm.setVoltage(volts);
    }
}
