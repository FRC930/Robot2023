package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeRollerMotorSubsystem;

/*
 * <h3>ExtendIntakeCommand</h3>
 * 
 * Rotates the intake rollers based on a given voltage
 */
public class IntakeRollerCommand extends CommandBase{
    private IntakeRollerMotorSubsystem m_IntakeRollerMotorSubsystem;
    private double m_voltage;
    
    /**
     * <h3>ExtendIntakeCommand</h3>
     * 
     * Rotates the intake rollers based on the given voltage
     * 
     * @param voltage Gets voltage to determine speed and retract or Extend
     * @param IntakeRollerMotorSubsystem Roller motor subsystem
     */
    public IntakeRollerCommand(int voltage, IntakeRollerMotorSubsystem IntakeRollerMotorSubsystem ){
        m_voltage = voltage;
        m_IntakeRollerMotorSubsystem = IntakeRollerMotorSubsystem;
        addRequirements(m_IntakeRollerMotorSubsystem);    
    } 

    @Override
    public void initialize() {
        m_IntakeRollerMotorSubsystem.setRollerVoltage(m_voltage);
    }

    @Override
    public void execute(){   
        Logger.getInstance().recordOutput("IntakeRollerMotorcommand/Voltage", m_voltage);
    }

    @Override
    public boolean isFinished(){ 
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeRollerMotorSubsystem.setRollerVoltage(0.0);     
    }    
    
}
