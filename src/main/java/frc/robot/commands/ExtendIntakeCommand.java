package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendIntakeMotorSubsystem;

/**
 * <h3>ExtendIntakeCommand</h3>
 * 
 * Extends or retracts the intake based on the given voltage
 */
public class ExtendIntakeCommand extends CommandBase{
   private ExtendIntakeMotorSubsystem m_ExtendIntakeMotorSubsystem;
   private double m_voltage;
    

    /**
     * <h3>ExtendIntakeCommand</h3>
     * 
     * Extends or retracts the intake based on the given voltage
     * 
     * @param voltage Gets voltage to determine speed and retract or Extend
     * @param extendIntakeMotorSubsystem Extend intake motor subsystem
     */
    public ExtendIntakeCommand(int voltage, ExtendIntakeMotorSubsystem extendIntakeMotorSubsystem){
        m_voltage = voltage;
        m_ExtendIntakeMotorSubsystem = extendIntakeMotorSubsystem;
        addRequirements(m_ExtendIntakeMotorSubsystem);
        
    } 
     @Override
    public void initialize() {
        m_ExtendIntakeMotorSubsystem.setVoltage(m_voltage);
    }

    @Override
    public void execute(){   
        Logger.getInstance().recordOutput("ExtendIntakeMotorCommand/Voltage", m_voltage);
    }

    @Override
    public boolean isFinished(){ 
        return false;
    }
}
