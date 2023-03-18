package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkMaxWrapper;

/**
 * 
 * <h3>IntakeRollerMotorSubsystem</h3>
 * 
 * Moves the roller motors
 * 
 */
public class IntakeRollerMotorSubsystem extends SubsystemBase{
    
    // -------- CONSTANTS --------\\
    private final int m_freeLimit = 30;
    private final int m_stallLimit = 10;
    

    private SparkMaxWrapper m_IntakerollerMotor;

    /**
     * 
     * <h3>IntakeRollerMotorSubsystem</h3>
     * 
     * Moves the roller motors
     * 
     * @param rollerID Can ID of the Intake Roller Motor
     */
    public IntakeRollerMotorSubsystem(int rollerID) {
        //Creates the motor
        m_IntakerollerMotor = new SparkMaxWrapper(rollerID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_IntakerollerMotor.setSmartCurrentLimit(m_stallLimit, m_freeLimit);

        m_IntakerollerMotor.restoreFactoryDefaults();
    }

    /**
     *
     * <h3>setRollerMotorSpeed</h3>
     * 
     * Sets the motor speed
     *  
     * @param speed
     */
    public void setRollerVoltage(double voltage) {
        m_IntakerollerMotor.setVoltage(-voltage);
    }

    /**
     *
     * <h3>stopRollerMotor</h3>
     * 
     * sets roller speed to 0
     * 
     */
    public void stopRollerMotor() {
        m_IntakerollerMotor.set(0.0);
    }

}