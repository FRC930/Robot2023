package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.CommandFactoryUtility;

public class ManipulatorSubsystem extends SubsystemBase {
    
    private final ProfiledPIDController controller;
    private final ArmFeedforward ff;
    private double targetPosition;
    private final ManipulatorIO m_io;
    public static double HIGH_POSITION = CommandFactoryUtility.MANIPULATOR_HIGH_SCORE; //at high Manipulator position
    public static double MEDIUM_POSITION = CommandFactoryUtility.MANIPULATOR_MID_SCORE; //at medium manipulator position
    public static double GROUND_POSITION = CommandFactoryUtility.MANIPULATOR_LOW_SCORE; //at ground manipulator position
    public static double STOW_POSITION = 45.0; //at stow manipulator angle
    public static double INTAKE_POSITION = CommandFactoryUtility.MANIPULATOR_INTAKE; // low intake Position
    public static final double SUBSTATION_POSITION = CommandFactoryUtility.MANIPULATOR_SUBSTATION;//-125; want position to force long way if continuousinput commented out

    public static final double ROLLER_INTAKE_SPEED = 0.8; //1.0 larger gear
    public static final double DOUBLE_SUBSTATION_ROLLER_INTAKE_SPEED = 0.7; //1.0 larger gear
    public static final double SHOOT_SPEED = -0.6;
    public static final double RELEASE_SPEED = -0.35; //-0.7 larger gear //
    public static final double HOLD_SPEED = 0.15; //0.25 larger gear
   

    /**<h3>ManipulatorSubsystem</h3>
     * Decides desired output, in volts, for the manipulator.
     * @param io The ArmIO, use IORobot if robot is real, otherwise use IOSim.
     */
    public ManipulatorSubsystem (ManipulatorIO io) {

        // Sets up PID controller TODO: Change these values
        // controller = new ProfiledPIDController(0.2, 0, 0, new Constraints(360, 720));
        controller = new ProfiledPIDController(0.2, 0, 0.02, new Constraints(540, 720)); //0.2
        controller.setTolerance(1, 1);
        //controller.enableContinuousInput(0, 360); // commented out for substation want to go long way!!

        // Sets up Feetforward TODO: Change these values
        ff = new ArmFeedforward(0.0, 0.35, 0); //g 0.35

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
            
            //Set up Feed Forward
            double feedforward = ff.calculate(Units.degreesToRadians(currentDegrees), Units.degreesToRadians(m_io.getVelocityDegreesPerSecond()));


            effort += feedforward;
            effort = MathUtil.clamp(effort, -8, 8);

            m_io.setVoltage(effort);
            
            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/Effort", effort);

            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/FeedForward", feedforward);
        } else {
            controller.reset(m_io.getCurrentAngleDegrees());
        }

        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/TargetPosition", targetPosition);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/EncoderValue", getPosition());
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/RawEncoderValue", getRawPosition());
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
    
    public double getRawPosition(){
        return  m_io.getRealCurrentAngleDegrees();
    }

    /**<h3>getRollerVoltage</h3>
     * Gets the voltage of the roller
     * @return getRollerVolate
     */
    public double getRollerVoltage() {
        return m_io.getRollerVoltage();
    }

    public double getRollerCurrent(){
        return m_io.getRollerCurrent();
    }

    /**<h3>getRollerSpeed</h3>
     * Sets the roller speed
     * @return setRollerSpeed
     */
    public void setRollerSpeed(double speed) {
        m_io.setRollerSpeed(speed);
        Logger.getInstance().recordOutput("RunManipulatorRollerCommand/ManipulatorSpeed", speed);
    }

    public Command setWristPositionCommand(double degrees) {
        return new InstantCommand(() -> setPosition(degrees), this);
    }

    private boolean atSetPoint() {
        return this.controller.atGoal();
    }

    public Command createWaitUntilAtAngleCommand() {
        return Commands.waitUntil(() -> this.atSetPoint());
    }

    public Command waitUntilCurrentPast(double amps) { 
        Debouncer debouncer = new Debouncer(.02); //Creates a debouncer to confirm amps are greater than value for .1 seconds
        return Commands.waitUntil(() -> debouncer.calculate(this.getRollerCurrent() > amps));
    }
}