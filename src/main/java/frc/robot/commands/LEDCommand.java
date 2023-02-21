package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDsubsystem;


public class LEDCommand extends CommandBase {
    //----Declarations----\\
    int pin0;
    int pin1;
    int pin2;
    int pin3;

    private LedPatterns m_pattern;
    private LEDsubsystem m_LEDSubsystem;

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

    /**
     * <h3>LEDCommand</h3>
     * Create LED command and pattern and assign variables to values
     * @param ledSubsystem
     * @param pattern
     */
    public LEDCommand(LEDsubsystem ledSubsystem, LedPatterns pattern){
        m_pattern = pattern;
        m_LEDSubsystem = ledSubsystem;
        addRequirements (m_LEDSubsystem);
    }
    /**
     * <h3>end</h3>
     * Ends LED patterns
     * @param interrupted
     */
    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
    }
    /**
     * <h3>initalize</h3>
     * Intialize all the LED commands and set up thier correct DIO pins
     */
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
    /**
     * <h3>isFinished</h3>
     * Ends the commands when done
     * @return isFinished
     */
    @Override
    public boolean isFinished(){
        return false;
    }

}
