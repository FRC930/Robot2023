package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Preferences;

public class ArmIORobot implements ArmIO {

    private CANSparkMax m_armMotor;
    
    private AbsoluteEncoder m_armEncoder;

    private static final String PREFERENCE_NAME = "ArmOffsetDegrees";
    private static double m_armOffsetDegrees = Preferences.getDouble(PREFERENCE_NAME, -12.0);

    // 6.3 encoder value is 0
    private static double m_armOffset = 36.0; //119.0; //1.3-12.0 this is minus 10.0  // 92.7+36.5;
    

    public ArmIORobot(int armMotorID) {
        m_armMotor = new CANSparkMax(armMotorID, MotorType.kBrushless);

        m_armMotor.restoreFactoryDefaults(); 
        m_armMotor.setIdleMode(IdleMode.kBrake);
        m_armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        m_armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        // Initializes Absolute Encoders from motors
        m_armEncoder = m_armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        // Sets position and velocity conversion factors so units are in degrees and degrees/second
        m_armEncoder.setPositionConversionFactor(360);
        m_armEncoder.setVelocityConversionFactor(60);

        m_armEncoder.setInverted(true);
        m_armMotor.setInverted(true);

        m_armEncoder.setZeroOffset((m_armOffset+m_armOffsetDegrees)%360);
    }

    /**
     * <h3>updateInputs</h3>
     * 
     * Left blank because it's only used in simulation
     */
    @Override
    public void updateInputs() {}

    /**
     * <h3>getOutputVoltage</h3>
     * 
     * Gets the shoulder motor outputs in volts
     * @return the sholder motor output voltage
     */
    @Override
    public double getCurrentAngleDegrees() {
        return m_armEncoder.getPosition();
    }

    /**
     * <h3>getVelocityDegreesPerSecond</h3>
     * 
     * Gets the shoulder motor's velocity
     * @return velocity of arm motor
     */
    @Override
    public double getVelocityDegreesPerSecond() {
        return m_armEncoder.getVelocity();
    }

    /**
     * <h3>setVoltage</h3>
     * 
     * Set the shoulder motor voltage 
     * @param volts desired voltage
     */
    @Override
    public void setVoltage(double volts) {
        m_armMotor.setVoltage(volts);
    }

    @Override
    public void adjustOffsetDegrees(double offsetDegrees) {
        m_armOffsetDegrees += offsetDegrees;
        Preferences.setDouble(PREFERENCE_NAME, m_armOffsetDegrees);
        m_armEncoder.setZeroOffset ((m_armOffset + m_armOffsetDegrees) % 360);
    }
}
