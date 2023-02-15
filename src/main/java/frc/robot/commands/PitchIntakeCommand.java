package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.rotateintake.PitchIntakeSubsystem;

/**
 * <h3>PitchIntakeCommand</h3>
 * 
 * Moves the PitchIntakeMotor
 * 
 */
public class PitchIntakeCommand extends CommandBase{
    private double desiredPosition;
    private PIDController pitchController;
    private PitchIntakeSubsystem m_PitchIntakeSubsystem;
    /**
     * 
     * <h3>PitchIntakeCommand</h3>
     * 
     * Moves the PitchIntakeMotor
     * 
     * @param pitchIntakeSubsystem The PitchIntakeMotor
     * @param voltage The desired voltage for the motor
    */
    public PitchIntakeCommand(PitchIntakeSubsystem PitchIntakeSubsystem, double desiredPosition) {
        this.desiredPosition = desiredPosition;
        m_PitchIntakeSubsystem = PitchIntakeSubsystem;
        addRequirements(m_PitchIntakeSubsystem);
        pitchController = new PIDController(1, 0, 0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double requiredAngle = pitchController.calculate(m_PitchIntakeSubsystem.getEncoderPosition(), desiredPosition);
        Logger.getInstance().recordOutput("RequiredAngle", requiredAngle);
        double requiredVoltage = requiredAngle/32;
        Logger.getInstance().recordOutput("RequiredVoltage", requiredVoltage);
        m_PitchIntakeSubsystem.setMotorVoltage(requiredVoltage);        
        Logger.getInstance().recordOutput("AutoBalanceCommand/currentPosition", m_PitchIntakeSubsystem.getEncoderPosition());
        Logger.getInstance().recordOutput("DesiredPosistion", desiredPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
