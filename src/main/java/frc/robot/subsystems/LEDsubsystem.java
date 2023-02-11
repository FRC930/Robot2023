package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDsubsystem extends SubsystemBase {

    private DigitalOutput m_pin1;
    private DigitalOutput m_pin2;
    private DigitalOutput m_pin3;
    private DigitalOutput m_pin4;


    public LEDsubsystem(int deviceAddress1, int deviceAddress2, int deviceAddress3, int deviceAddress4 ) {
        m_pin1 = new DigitalOutput(deviceAddress1);
        m_pin2 = new DigitalOutput(deviceAddress2);
        m_pin3 = new DigitalOutput(deviceAddress3);
        m_pin4 = new DigitalOutput(deviceAddress4);
        
    }

    public  void clear(){
        m_pin1.set(false);
        m_pin2.set(false);
        m_pin3.set(false);
        m_pin4.set(false);
       
    }
    public  void setPins(boolean pin1, boolean pin2, boolean pin3, boolean pin4){
        m_pin1.set(pin1);
        m_pin2.set(pin2);
        m_pin3.set(pin3);
        m_pin4.set(pin4);
        
    }    
}