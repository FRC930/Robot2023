package frc.robot.utilities;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;

public class TimeOfFlightUtility {

    //-------- CONSTANTS --------\\

    private final double TRIGGER_DISTANCE = 350.0; // TODO: Finalize trigger distance
    
    //-------- VARIABLES --------\\
    
    private DigitalInput m_sensorSim;

    private TimeOfFlight m_sensor;

    private Debouncer m_debouncer;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>TimeOfFlightUtility</h3>
     * 
     * Utility to find whether sensor has detected game object.
     * 
     * @param sensorID - ID of sensor
     */
    public TimeOfFlightUtility(int sensorID) {
        // TODO: Decide debounce time
        m_debouncer = new Debouncer(0.05);
        if (Robot.isReal()) {
            m_sensor = new TimeOfFlight(sensorID);
            m_sensor.setRangingMode(RangingMode.Medium, 20);
        } else {
            m_sensorSim = new DigitalInput(sensorID);
        }
    }

    //-------- METHODS --------\\

    /**
     * <h3>sensorDetected</h3>
     * 
     * This method returns the sensor's value
     * 
     * @return Whether or not sensor has detected game object
     */
    public boolean sensorDetected() {
        if (Robot.isReal()) {
            return m_debouncer.calculate(m_sensor.getRange() < TRIGGER_DISTANCE);
        } else {
            return m_sensorSim.get();
        }
    }

} // End of class IndexerSensorUtility