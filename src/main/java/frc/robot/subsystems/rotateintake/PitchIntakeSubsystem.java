package frc.robot.subsystems.rotateintake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <h3>RotateIntakeRollerMotorSubsystem</h3>
 * 
 * Moves the Rotate Intake Roller Motor
 * 
 */
public class PitchIntakeSubsystem extends SubsystemBase{
    
    private IntakeMotorIO m_RotateIntakerollerMotorIO;
    
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
        Logger.getInstance().recordOutput("EncoderPosition", m_RotateIntakerollerMotorIO.getCurrentAngleDegrees());
        return m_RotateIntakerollerMotorIO.getCurrentAngleDegrees();
        // TODO should we be using set current limit
    }

    /**
     * 
     * <h3>periodic</h3>
     * 
     * Periodically updates the input in the rotate intake roller motor
     * 
     */
    @Override
    public void periodic() {
        this.m_RotateIntakerollerMotorIO.updateInputs();
    }
}