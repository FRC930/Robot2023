package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.rotateintake.PitchIntakeSubsystem;
import frc.robot.utilities.DesiredPitchUtility;

/**
 * 
 * <h3>MonitorPitchIntakeCommand</h3>
 * 
 * Stops the PitchIntakeMotor when its arrived at its desired state
 * 
 */
public class MonitorPitchIntakeCommand extends CommandBase{
    private final double deadBand = 0.0;
    private PIDController pitchController;
    private PitchIntakeSubsystem m_PitchIntakeSubsystem;

    /**
     * 
     * <h3>MonitorPitchIntakeCommand</h3>
     * 
     * Stops the PitchIntakeMotor when its arrived at its desired state
     * 
     * @param pitchIntakeSubsystem The PitchIntakeMotor
     * @param desiredPosition The desired position of the motor
     */
    public MonitorPitchIntakeCommand(PitchIntakeSubsystem pitchIntakeSubsystem) {
        m_PitchIntakeSubsystem = pitchIntakeSubsystem;
        addRequirements(m_PitchIntakeSubsystem);
        pitchController = new PIDController(1, 0, 0);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double requiredAngle = pitchController.calculate(m_PitchIntakeSubsystem.getEncoderPosition(), DesiredPitchUtility.getInstance().getDesiredPosition());
        Logger.getInstance().recordOutput("RequiredAngle", requiredAngle);
        double requiredVoltage = requiredAngle/32;
        Logger.getInstance().recordOutput("RequiredVoltage", requiredVoltage);
        m_PitchIntakeSubsystem.setMotorVoltage(requiredVoltage);        
        Logger.getInstance().recordOutput("AutoBalanceCommand/currentPosition", m_PitchIntakeSubsystem.getEncoderPosition());
        Logger.getInstance().recordOutput("DesiredPosistion", DesiredPitchUtility.getInstance().getDesiredPosition());
    }

    @Override
    public boolean isFinished() {
       return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
