package frc.robot.subsystems.rotateintake;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/**
 * <h3>RotateIntakeRollerMotorSubsystem</h3>
 * 
 * Moves the Rotate Intake Roller Motor
 * 
 */
public class PitchIntakeSubsystem extends SubsystemBase{
    
    private final ProfiledPIDController controller;
    private final ArmFeedforward ff;

    private double targetPosition;

    private IntakeMotorIO m_RotateIntakerollerMotorIO;
    
    /**
     * <h3>PitchIntakeSubsystem</h3>
     * 
     * Moves the RotateIntakeRollerMotor
     * 
     * @param motorID Can ID of the Rotate Intake Roller Motor
     */
    public PitchIntakeSubsystem(IntakeMotorIO intakeMotorIO) {
        m_RotateIntakerollerMotorIO = intakeMotorIO;

        // Sets up PID controller TODO: Change these values
        //controller = new ProfiledPIDController(0.35, 0, 0, new Constraints(50, 50));
        controller = new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(360, 720));
        controller.setTolerance(1, 1);
        controller.enableContinuousInput(0, 360);

        // Sets up Feetforward TODO: Change these values
        ff = new ArmFeedforward(0.0, 0.0, 0.0);
    }

    /**
     *
     * <h3>setMotorVoltage</h3>
     * 
     * Sets the motors voltage
     * 
     * @param voltage the voltage given to the motor
     */
    public void setMotorVoltage(double voltage) {
        m_RotateIntakerollerMotorIO.setVoltage(voltage);
    }

    /**<h3>setPosition</h3>
     * Moves the pitch intake to the desired position, using voltage.
     * @param target Desired pitch intake position in degrees
     */
    public void setPosition(double target) {
        targetPosition = target;
    }

    /**
     *
     * <h3>getEncoderPosition</h3>
     * 
     * Gets the current encoder position
     * 
     */
    public double getEncoderPosition() {
        //Logger.getInstance().recordOutput("EncoderPosition", m_RotateIntakerollerMotorIO.getCurrentAngleDegrees());
        SmartDashboard.putNumber("PitchIntakeEncoderPosition", m_RotateIntakerollerMotorIO.getCurrentAngleDegrees());
        return m_RotateIntakerollerMotorIO.getCurrentAngleDegrees();
        // TODO should we be using set current limit
    }

    /**
     * 
     * <h3>periodic</h3>
     * 
     * Periodically updates the input in the rotate intake roller motor
     * 
     */
    @Override
    public void periodic() {
        this.m_RotateIntakerollerMotorIO.updateInputs();
        Logger.getInstance().recordOutput("AutoBalanceCommand/currentPosition", getEncoderPosition());
        
        if (DriverStation.isEnabled() || !Robot.isReal()){
            
            m_RotateIntakerollerMotorIO.updateInputs();

            double currentDegrees = m_RotateIntakerollerMotorIO.getCurrentAngleDegrees();

            // Set up PID controller
            double effort = controller.calculate(currentDegrees, targetPosition);
            
            //Set up Feed Forward
            double feedforward = ff.calculate(Units.degreesToRadians(currentDegrees), Units.degreesToRadians(m_RotateIntakerollerMotorIO.getCurrentVelocityDegreesPerSecond()));


            effort += feedforward;
            effort = MathUtil.clamp(effort, -8, 8);

            m_RotateIntakerollerMotorIO.setVoltage(effort);
            
            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/Effort", effort);

            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/Feed Forward", feedforward);
            
        } else {
            controller.reset(m_RotateIntakerollerMotorIO.getCurrentAngleDegrees());
        }
        SmartDashboard.putNumber("PitchIntakeSubsystem/currentDegrees", getEncoderPosition());
    }
}