package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkMaxWrapper;

public class ExtendIntakeMotorSubsystem extends SubsystemBase{

     // -------- CONSTANTS --------\\
    private final int m_freeLimit = 10;
    private final int m_stallLimit = 30;

    // -------- DECLARATIONS --------\\
    private final SparkMaxWrapper m_intakeMotor;
    

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>IntakeEncoderSubsystem</h3>
     * Creates a subsystem class to manage the intake motor.
     * 
     * @param intakeMotorID ID of the intake motor
     */
    public ExtendIntakeMotorSubsystem(int intakeMotorID) {
        
        m_intakeMotor = new SparkMaxWrapper(intakeMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        //TODO change current limit based on whether we are trying to hold intake down or moving intake
        m_intakeMotor.setSmartCurrentLimit(m_stallLimit, m_freeLimit);

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(m_intakeMotor, DCMotor.getNEO(12));
            
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
    public void simulationPeriodic() {
      REVPhysicsSim.getInstance().run();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ExtendIntakeMotorSubsystem/OutputAmps", m_intakeMotor.getOutputCurrent());
    }
}