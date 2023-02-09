package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendIntakeMotorSubsystem;


public class ExtendIntakeCommand extends CommandBase{
   private ExtendIntakeMotorSubsystem m_ExtendIntakeMotor;
    private double m_voltage;
    private ExtendIntakeMotorSubsystem ExtendIntakeMotorSubsystem;

    /**
     * <h3>ExtendIntakeCommand</h3>
     * retracts and Extends the Intake
     * 
     * @param voltage //gets voltage to determine speed and retract or Extend
     */
    public ExtendIntakeCommand(int voltage){
        m_voltage = voltage;
        m_ExtendIntakeMotor = ExtendIntakeMotorSubsystem;
    } 
     @Override
    public void initialize() {
        m_ExtendIntakeMotor.setVoltage(m_voltage);
    }

    @Override
    public void execute(){   
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    public void setDefaltCommand(ExtendIntakeCommand extendIntakeCommand) {
    }

  
  
}
