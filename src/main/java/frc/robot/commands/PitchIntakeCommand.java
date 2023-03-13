package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.rotateintake.PitchIntakeSubsystem;

/**
 * <h3>PitchIntakeCommand</h3>
 * 
 * Moves the PitchIntakeMotor
 */
public class PitchIntakeCommand extends CommandBase{
    private PIDController m_pitchController;

    private PitchIntakeSubsystem m_PitchIntakeSubsystem;
    private double m_desiredPosition;

    private double m_requiredAngle;
    private double m_requiredVoltage;

    /**
     * <h3>PitchIntakeCommand</h3>
     * 
     * Moves the PitchIntakeMotor
     * 
     * @param pitchIntakeSubsystem The PitchIntakeMotor
     * @param desiredPosition The desired position for the motor
    */
    public PitchIntakeCommand(PitchIntakeSubsystem PitchIntakeSubsystem, double desiredPosition) {
        m_desiredPosition = desiredPosition;
        m_PitchIntakeSubsystem = PitchIntakeSubsystem;
        addRequirements(m_PitchIntakeSubsystem);

        // TODO find the actual PID values
        m_pitchController = new PIDController(1, 0, 0);
    }

    @Override
    public void initialize() {
        m_PitchIntakeSubsystem.setPosition(m_desiredPosition);
        Logger.getInstance().recordOutput("DesiredPosistion", m_desiredPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
