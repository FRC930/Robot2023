package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

public class ArmSubsystem extends SubsystemBase {

    
    private final ProfiledPIDController controller;
    private final ArmFeedforward ff;
    private final ArmIO m_armIO;

    private double targetPosition;

    //TODO use offsets for positions
    public static double HIGH_POSITION = CommandFactoryUtility.ARM_HIGH_SCORE_ANGLE; //at high arm position
    public static double MEDIUM_POSITION = CommandFactoryUtility.ARM_MID_SCORE_ANGLE; //at medium arm position
    public static double GROUND_POSITION = CommandFactoryUtility.ARM_LOW_SCORE_ANGLE; //at ground arm position
    public static double STOW_POSITION = 84.0;//-60.0; //at arm Stow Position
    public static double INTAKE_POSITION = CommandFactoryUtility.ARM_INTAKE_ANGLE; //for low intake Position
    public static final double SUBSTATION_POSITION = CommandFactoryUtility.ARM_SUBSTATION_ANGLE;// TODO:find acutal position

    public static double ARM_LENGTH = 27.12;

    /**
     * <h3>ArmSubsystem</h3>
     * 
     * Controls the motors and encoders, shoulder and wrist, on the arm.
     * 
     * @param io The ArmIO, use IORobot if robot is real, otherwise use IOSim.
     */
    public ArmSubsystem (ArmIO armIO) {

        // Sets up PID controller
        // controller = new ProfiledPIDController(0.2, 0, 0.02, new Constraints(225, 270));
        controller = new ProfiledPIDController(0.25, 0, 0.025, new Constraints(225, 360)); //0.25

        controller.setTolerance(1, 1);
        controller.enableContinuousInput(0, 360);

        // TODO Change values when manipulator is added
        ff = new ArmFeedforward(0, 1.0, 0);
        
        m_armIO = armIO;

        targetPosition = STOW_POSITION;
    }

    /**
     * Gets the inputs from the IO, and uses the feed forward and the PID controller to calculate the effort, in volts, to set to the io,
     * for both the shoulder and wrist motors.
     */
    @Override
    public void periodic() {
        m_armIO.updateInputs();

        // caculate PID and Feet forward angles 
        if (DriverStation.isEnabled() || !Robot.isReal()) {
            double effort = controller.calculate(m_armIO.getCurrentAngleDegrees(), targetPosition);
            double feedforward = ff.calculate(Units.degreesToRadians(m_armIO.getCurrentAngleDegrees()), Units.degreesToRadians(m_armIO.getVelocityDegreesPerSecond()));

            effort += feedforward;
            effort = MathUtil.clamp(effort, -6, 6);

            m_armIO.setVoltage(effort);

            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/FeedForward", feedforward);
            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/Effort", effort);
            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/Error", controller.getPositionError());
        }
        else{
            controller.reset(m_armIO.getCurrentAngleDegrees());
        }
        
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/TargetPosition", targetPosition);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/EncoderValue", getPosition());
    }

    /**
     * <h3>setPosition</h3>
     * 
     * Moves the shoulder to the desired position, using voltage.
     * 
     * @param target Desired shoulder position in degrees
     */
    public void setPosition(double target) {
        targetPosition = target;
    }

    /**
     * <h3>getPosition</h3>
     * 
     * Gets the Shoulder motor position in degrees
     */
    public double getPosition(){
        return m_armIO.getCurrentAngleDegrees();
    }

    public Command setArmPositionCommand(double degrees) {
        return new InstantCommand(() -> setPosition(degrees), this);
    }

    private boolean atSetPoint() {
        return this.controller.atGoal();
    }

    private boolean lessThan(double angle) {
        return this.getPosition() < angle;
    }

    private boolean greaterThan(double angle) {
        return this.getPosition() > angle;
    }

    public Command createWaitUntilAtAngleCommand() {
        return Commands.waitUntil(() -> this.atSetPoint());
    }

    public Command createWaitUntilLessThanAngleCommand(double angle) {
        return Commands.waitUntil(() -> this.lessThan(angle));
    }

    public Command createWaitUntilGreaterThanAngleCommand(double angle) {
        return Commands.waitUntil(() -> this.greaterThan(angle));
    }
}
