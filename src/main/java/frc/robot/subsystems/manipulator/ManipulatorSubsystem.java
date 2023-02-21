package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ManipulatorSubsystem extends SubsystemBase {
    
    private final ProfiledPIDController controller;
    private final ArmFeedforward ff;
    private double targetPosition;
    private final ManipulatorIO m_io;
    public static double HIGH_POSITION = 27.6; //at high elevator position
    public static double MEDIUM_POSITION = 23.9; //at medium elevator position
    public static double GROUND_POSITION = 5.2; //at ground elevator position
    public static double STOW_POSITION = 45.0; //at ground elevator position
    public static double INTAKE_POSITION = -225.0; //TODO: Find actual intake position value

    public static final double ROLLER_INTAKE_SPEED = 0.8;
    public static final double RELEASE_SPEED = -0.3;

    /**<h3>ManipulatorSubsystem</h3>
     * Decides desired output, in volts, for the manipulator.
     * @param io The ArmIO, use IORobot if robot is real, otherwise use IOSim.
     */
    public ManipulatorSubsystem (ManipulatorIO io) {

        // Sets up PID controller TODO: Change these values
        //controller = new ProfiledPIDController(0.35, 0, 0, new Constraints(50, 50));
        controller = new ProfiledPIDController(0.2, 0, 0, new Constraints(360, 720));
        controller.setTolerance(1, 1);

        // Sets up Feetforward TODO: Change these values
        ff = new ArmFeedforward(0.0, 0.7, 0);

        m_io = io;

        targetPosition = STOW_POSITION;
    }

    /**<h3>periodic</h3>
    * Gets the inputs from the IO, and uses the feed forward and the PID controller to calculate the effort, in volts, to set to the io,
    * for the manipulator motor.
     */
    @Override
    public void periodic() {

        if (DriverStation.isEnabled() || !Robot.isReal()){
            
            this.m_io.updateInputs();

            double currentDegrees = m_io.getCurrentAngleDegrees();

            // Set up PID controller
            double effort = controller.calculate(currentDegrees, targetPosition);
            controller.setTolerance(1, 1);
            controller.enableContinuousInput(0, 360);
            
            //Set up Feed Forward
            double feedforward = ff.calculate(Units.degreesToRadians(currentDegrees), Units.degreesToRadians(m_io.getVelocityDegreesPerSecond()));


            effort += feedforward;
            effort = MathUtil.clamp(effort, -8, 8);

            m_io.setVoltage(effort);
            
            SmartDashboard.putNumber("MANIPULATOR EFFORT", effort);

            SmartDashboard.putNumber("MANIPULATOR FEED FORWARD", feedforward);
        } else {
            controller.reset(m_io.getCurrentAngleDegrees());
        }

        SmartDashboard.putNumber("MANIPULATOR TARGET POSITION", targetPosition);
        SmartDashboard.putNumber("Manipulator Encoder Value", getPosition());
    }

    /**<h3>setPosition</h3>
     * Moves the manipulator to the desired position, using voltage.
     * @param target Desired manipulator position in degrees
     */
    public void setPosition(double target) {
        targetPosition = target;
    }
    
    /**<h3>getPosition</h3>
     * Gets the manipulator motor position in degrees
     * @return getCurrentAngleDegrees
     */
    public double getPosition(){
        return m_io.getCurrentAngleDegrees();
    }

    /**<h3>getRollerVoltage</h3>
     * Gets the voltage of the roller
     * @return getRollerVolate
     */
    public double getRollerVoltage() {
        return m_io.getRollerVoltage();
    }

    /**<h3>getRollerSpeed</h3>
     * Sets the roller speed
     * @return setRollerSpeed
     */
    public void setRollerSpeed(double speed) {
        m_io.setRollerSpeed(speed);
    }

    public Command setWristPositionCommand(double degrees) {
        return new InstantCommand(() -> setPosition(degrees), this);
    }
}
