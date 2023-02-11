package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendIntakeMotorSubsystem;


public class ExtendIntakeCommand extends CommandBase{
   private ExtendIntakeMotorSubsystem m_ExtendIntakeMotor;
   private double m_voltage;
    

    /**
     * <h3>ExtendIntakeCommand</h3>
     * retracts and Extends the Intake
     * 
     * @param voltage //gets voltage to determine speed and retract or Extend
     */
    public ExtendIntakeCommand(int voltage, ExtendIntakeMotorSubsystem ExtendIntakeMotor){
        m_voltage = voltage;
        m_ExtendIntakeMotor = ExtendIntakeMotor;
        addRequirements(m_ExtendIntakeMotor);
        
    } 
     @Override
    public void initialize() {
        m_ExtendIntakeMotor.setVoltage(m_voltage);
    }

    @Override
    public void execute(){   
        Logger.getInstance().recordOutput("ExtendIntakeMotorCommand/Voltage", m_voltage);
    }

    @Override
    public boolean isFinished(){ 
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
