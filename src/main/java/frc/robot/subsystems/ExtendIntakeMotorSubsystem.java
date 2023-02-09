package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtendIntakeMotorSubsystem extends SubsystemBase{
    
    private CANSparkMax m_ExtendIntakeMotor;
     // -------- CONSTANTS --------\\
    private final int m_freeLimit = 65;
    private final int m_stallLimit = 10;

    // -------- DECLARATIONS --------\\
    private final CANSparkMax m_intakeMotor;
    

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>IntakeEncoderSubsystem</h3>
     * Creates a subsystem class to manage the intake motor.
     * 
     * @param intakeMotorID ID of the intake motor
     */
    public ExtendIntakeMotorSubsystem(int intakeMotorID) {

        
        m_intakeMotor =  new CANSparkMax(intakeMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_intakeMotor.setSmartCurrentLimit(m_stallLimit, m_freeLimit);

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(m_ExtendIntakeMotor, DCMotor.getNEO(12));
            
          }

        // Sets motor so it can't be manually moved when neutral
        m_intakeMotor.setIdleMode(IdleMode.kBrake);

        // Motor is not inverted
        m_intakeMotor.setInverted(false);

    }
    /**
     * <h3> extendIntake</h3>
     * It runs the intake and extends it by setting voltage
     */
    public void setVoltage(double voltage) {
         // sets the voltage boundries
         m_intakeMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("IntakeMotorSubsystem/current", m_intakeMotor.getOutputCurrent());
    }
    
    @Override
    public void simulationPeriodic() {
      REVPhysicsSim.getInstance().run();
    }
}