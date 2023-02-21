package frc.robot.utilities;
//Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotController;

//This class manages spark max speed and id's
public class SparkMaxWrapper extends CANSparkMax {
    private SimDouble m_simSpeed;
    private SimDevice m_simSparkMax;

    /**
     * <h3>SparkMaxWrapper</h3>
     * 
     * Manages a spark max, allowing it to run in simulation.
     * @param deviceID - Can BUS ID of desired motor
     * @param type - Motor type, e.g. MotorType.kBrushless
     */
    public SparkMaxWrapper(int deviceID, MotorType type) {
        super(deviceID,type);

        m_simSparkMax = SimDevice.create("SparkMax",deviceID);
        if (m_simSparkMax != null){
            m_simSpeed = m_simSparkMax.createDouble("speed", SimDevice.Direction.kInput, 0.0);
        }
    }
    
    /**
     * <h3>get</h3>
     * 
     * pulls and returns speed from spark max.
     * @return speed of spark max
     */
    @Override
    public double get(){
        if (m_simSparkMax != null){
            return m_simSpeed.get();
        }
        return super.get();
    }

    /**
     * <h3>set</h3>
     * 
     * sets speed of spark max.
     * @param speed - Desired speed
     */
    @Override
    public void set(double speed){
        if (m_simSparkMax != null){
            m_simSpeed.set(speed);
        }else{
            super.set(speed);
        }
    }

    /**
     * <h3>setVoltage</h3>
     * 
     * Sets voltage of spark max.
     * @param outputVolts - Desired voltage
     */
    @Override
    public void setVoltage(double outputVolts) { //For simulation purposes, we are expecting that the battery voltage stays constant.
        if (m_simSparkMax != null){
            set(outputVolts / RobotController.getBatteryVoltage());
        } else {
            super.setVoltage(outputVolts);
        }
    }
}