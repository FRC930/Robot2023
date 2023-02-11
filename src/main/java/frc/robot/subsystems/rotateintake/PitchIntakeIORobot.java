package frc.robot.subsystems.rotateintake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

/**
 * 
 * <h3>PitchIntakeIORobot</h3>
 * 
 * Moves the PitchIntakeMotor
 * 
 */
public class PitchIntakeIORobot implements IntakeMotorIO {

    private CANSparkMax m_RotateIntakeRollerMotor;
    private RelativeEncoder m_RotateIntakeRollerEncoder;

    // TODO find actual values
    private final int m_freeLimit = 20;
    private final int m_stallLimit = 10;
    
    /**
     * 
     * <h3>PitchIntakeIORobot</h3>
     * 
     * Moves the PitchIntakeMotor
     * 
     * @param motorID ID of the motor
     * @param encoderID ID of the encoder
     */
    public PitchIntakeIORobot(int motorID) {
        //Creates the motor
        m_RotateIntakeRollerMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        m_RotateIntakeRollerMotor.restoreFactoryDefaults();

        //Creates the encoder
        m_RotateIntakeRollerEncoder = m_RotateIntakeRollerMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 42);
        m_RotateIntakeRollerEncoder.setPositionConversionFactor(360);
        // TODO Figure out what number the factor has to be
        m_RotateIntakeRollerEncoder.setVelocityConversionFactor(60);
        
        m_RotateIntakeRollerMotor.setSmartCurrentLimit(m_stallLimit, m_freeLimit);
    }

    /**
     * 
     * <h3>getOutputVoltage</h3>
     * 
     * Returns the motors voltage between -12 and 12
     * 
     */
    @Override
    public double getOutputVoltage() {
        return MathUtil.clamp(m_RotateIntakeRollerMotor.getBusVoltage(), -12, 12);
    }

    /**
     * <h3>getCurrentAngleDegrees</h3>
     * 
     * Returns motor's current position in degrees
     * 
     */
    @Override
    public double getCurrentAngleDegrees() {
        return m_RotateIntakeRollerEncoder.getPosition();
    }

    /**
     * <h3>getCurrentVelocityDegreesPerSecond</h3>
     * 
     * Returns the current velocity of the motor in degrees per second 
     * 
     */
    @Override
    public double getCurrentVelocityDegreesPerSecond() {
        return m_RotateIntakeRollerEncoder.getVelocity();
    }

    /**
     * <h3>getCurrentVelocityDegreesPerSecond</h3>
     * 
     * Sets the voltage for the intake roller motor 
     * 
     */
    @Override
    public void setVoltage(double volts) {
        m_RotateIntakeRollerMotor.setVoltage(volts);
    }
   
    @Override
    public void updateInputs() {}
    
}
