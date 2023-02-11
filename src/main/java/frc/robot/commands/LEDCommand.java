package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDsubsystem;


public class LEDCommand extends CommandBase {
    int pin0;
    int pin1;
    int pin2;
    int pin3;
    
    

    public static enum LedPatterns{
        CONEREQUEST,
        CUBEREQUEST,
        CONEAQUIRED,
        CUBEAQUIRED,
        BLUEALLIANCE,
        REDALLIANCE,
        DISABLED,
        TEAMCOLORS,
        RANDOMLED,
        AUTOBALANCE;
    }
    private LedPatterns m_pattern;
    private LEDsubsystem m_LEDSubsystem;

    public LEDCommand(LEDsubsystem ledSubsystem, LedPatterns pattern){
        m_pattern = pattern;
        m_LEDSubsystem = ledSubsystem;
        addRequirements (m_LEDSubsystem);
        // Do I need another addRequirment?
    }
    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
    }
    @Override
    public void initialize(){
        switch(m_pattern){
            case CONEREQUEST:
                m_LEDSubsystem.setPins(true, false, false, false);
                break;
            case CUBEREQUEST:
                m_LEDSubsystem.setPins(false, true, false, false);
                break;
            case CUBEAQUIRED:
                m_LEDSubsystem.setPins(false, false, true, false);
                break;
            case CONEAQUIRED:
                m_LEDSubsystem.setPins(false, false, false, true);
                break;
            case BLUEALLIANCE:
                m_LEDSubsystem.setPins(true, true, false, false);
                break;
            case REDALLIANCE:
                m_LEDSubsystem.setPins(true,true, true, false);
                break;
            case DISABLED:
                m_LEDSubsystem.setPins(false, false, false, false);
                break;
            case TEAMCOLORS:
                m_LEDSubsystem.setPins(true, true, true, true);
                break;
            case AUTOBALANCE:
                m_LEDSubsystem.setPins(true, false, true, false);
                break;
            case RANDOMLED:
                m_LEDSubsystem.setPins(false, false, true, true);
                break;

        }
    }
    @Override
    public boolean isFinished(){
        return true;
    }

 
    
}
