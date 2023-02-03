package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 
 * <h3>IntakeRollerMotorSubsystem</h3>
 * 
 * Moves the roller motors
 * 
 */
public class IntakeRollerMotorSubsystem extends SubsystemBase{
    
    private CANSparkMax m_IntakerollerMotor;

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
        m_IntakerollerMotor = new CANSparkMax(rollerID, MotorType.kBrushless);
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
    public void setRollerMotorSpeed(double speed) {
        m_IntakerollerMotor.set(speed);
    }

    /**
     *
     * <h3>setReversedRollerMotorSpeed</h3>
     * 
     * Sets the reversed motor speed 
     * 
     * @param speed the motors desired speed
     */
    public void setReversedRollerMotorSpeed(double speed) {
        m_IntakerollerMotor.set(-speed);
    }

    /**
     *
     * <h3>stopRollerMotor</h3>
     * 
     * Stops the rollers
     * 
     */
    public void stopRollerMotor() {
        m_IntakerollerMotor.set(0.0);
    }
}