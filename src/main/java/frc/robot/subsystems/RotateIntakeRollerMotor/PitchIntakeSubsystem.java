package frc.robot.subsystems.RotateIntakeRollerMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <h3>RotateIntakeRollerMotorSubsystem</h3>
 * 
 * Moves the Rotate Intake Roller Motor
 * 
 */
public class PitchIntakeSubsystem extends SubsystemBase{
    
    private IntakeMotorIO m_RotateIntakerollerMotorIO;
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
    public PitchIntakeSubsystem(IntakeMotorIO intakeMotorIO) {
        m_RotateIntakerollerMotorIO = intakeMotorIO;
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
        m_RotateIntakerollerMotorIO.setVoltage(voltage);
    }

    /**
     *
     * <h3>getEncoderPosition</h3>
     * 
     * Gets the current encoder position
     * 
     */
    public double getEncoderPosition() {
        return m_RotateIntakerollerMotorIO.getCurrentAngleDegrees();
    }

    @Override
    public void periodic() {
        this.m_RotateIntakerollerMotorIO.updateInputs();
    }
}