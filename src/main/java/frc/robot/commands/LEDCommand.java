package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private boolean m_canRunDisabled = false;

    public static enum LedPatterns{
        CONEREQUEST,
        CUBEREQUEST,
        CONEAQUIRED,
        CUBEAQUIRED,
        ALLIANCE,
        DISABLED,
        TEAMCOLORS,
        RANDOMLED,
        AUTOBALANCE;
    }

    /**
     * <h3>LEDCommand</h3>
     * Create LED command and pattern and assign variables to values
     * Sets m_canRunDisabled to True when the Disabled LED pattern is active in order to let it run when it disabled
     * @param ledSubsystem
     * @param pattern
     */
    public LEDCommand(LEDsubsystem ledSubsystem, LedPatterns pattern){
        m_pattern = pattern;
        m_LEDSubsystem = ledSubsystem;
        addRequirements(m_LEDSubsystem);

        if(pattern == LedPatterns.DISABLED){
            m_canRunDisabled = true;
        }
    }
    /**
     * <h3>end</h3>
     * Interrupts LED Patterns
     * @param interrupted
     */
    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
    }
    /**
     * <h3>initalize</h3>
     * Intialize all the LED commands and set up thier correct DIO pins (1 = true, 0 = false)
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
            case ALLIANCE:
                if(DriverStation.getAlliance() == Alliance.Blue){
                    // Blue Pins
                    m_LEDSubsystem.setPins(true, true, false, false);
                }
                else{
                    // Red Pina
                    m_LEDSubsystem.setPins(true,true, true, false);
                }
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
     * Ensures that the disabled pattern can run when the robot is disabled
     * @return m_canRunDisabled
     */
    
    @Override
    public boolean runsWhenDisabled() {
        return m_canRunDisabled;
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
