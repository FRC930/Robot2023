package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
        m_IntakerollerMotor = new SparkMaxWrapper(rollerID, MotorType.kBrushless);
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
        m_IntakerollerMotor.setVoltage(voltage);
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

    public void getSerialNumber() {
        // return m_IntakerollerMotor.getSerialNumber();
    }
}