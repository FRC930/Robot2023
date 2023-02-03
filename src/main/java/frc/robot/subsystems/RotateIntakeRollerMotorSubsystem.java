package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * <h3>RotateIntakeRollerMotorSubsystem</h3>
 * 
 * Moves the Rotate Intake Roller Motor
 * 
 */
public class RotateIntakeRollerMotorSubsystem extends SubsystemBase{
    
    private CANSparkMax m_RotateIntakeRollerMotor;
    private RelativeEncoder m_RotateIntakeRollerEncoder;
    // private CANSparkMax m_RotateIntakeRollerMotor;
    // private RelativeEncoder m_RotateIntakeRollerEncoder;
    
    /**
     * <h3>RotateIntakeRollerMotorSubsystem</h3>
     * 
     * Moves the RotateIntakeRollerMotor
     * 
     * @param motorID Can ID of the Rotate Intake Roller Motor
     * @param encoderID Can ID of the Rotate Intake Roller Encoder
     */
    public RotateIntakeRollerMotorSubsystem(int motorID, int encoderID) {
        //Creates the motor
        m_RotateIntakeRollerMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        m_RotateIntakeRollerMotor.restoreFactoryDefaults();

        //Creates the encoder
        m_RotateIntakeRollerEncoder = m_RotateIntakeRollerMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 42);
        m_RotateIntakeRollerEncoder.setPositionConversionFactor(360);
        //Figure out what number the factor has to be
        m_RotateIntakeRollerEncoder.setVelocityConversionFactor(60);
    }

    /**
     *
     * <h3>setMotorVoltage</h3>
     * 
     * Sets the motors voltage
     * 
     * @param voltage the voltage given to the motor
     */
    public void setMotorVoltage(double voltage) {
        // Converts degrees into encoder ticks
        m_RotateIntakeRollerMotor.setVoltage(voltage);
    }

    /**
     *
     * <h3>getEncoderPosition</h3>
     * 
     * Gets the current encoder position
     * 
     */
    public double getEncoderPosition() {
        return m_RotateIntakeRollerEncoder.getPosition();
    }
}