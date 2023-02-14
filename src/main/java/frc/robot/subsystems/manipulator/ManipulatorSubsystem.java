package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {
    
    private final ProfiledPIDController controller;
    private final ArmFeedforward ff;

    private double targetPosition;

    private final ManipulatorIO io;

    public static double highPosition = 27.6; //at high elevator position

    public static double mediumPosition = 23.9; //at medium elevator position

    public static double groundPosition = 5.2; //at ground elevator position

    public static double stowPosition = 90.0; //at ground elevator position

    //TODO: These are nonsensical (Fix once we get actual values)
    public static double intakePosition = -225.0;

    /**
     * Decides desired output, in volts, for the manipulator.
     * 
     * @param io The ArmIO, use IORobot if robot is real, otherwise use IOSim.
     */
    public ManipulatorSubsystem (ManipulatorIO io) {

        // Sets up PID controller TODO: Change these values
        controller = new ProfiledPIDController(0, 0, 0, new Constraints(360, 360));
        controller.setTolerance(1, 1);

        // Sets up Feetforward TODO: Change these values
        ff = new ArmFeedforward(0, 0, 0);

        this.io = io;

        //targetPosition = stowPosition;
    }

    /**
    * Gets the inputs from the IO, and uses the feed forward and the PID controller to calculate the effort, in volts, to set to the io,
    * for the manipulator motor.
     */
    @Override
    public void periodic() {
        this.io.updateInputs();
        // Set up PID controller
        double effort = controller.calculate(io.getCurrentAngleDegrees(), targetPosition);
        controller.setTolerance(1, 1);
        controller.enableContinuousInput(-180, 180);
        
        //Set up Feed Forward
        double feedforward = ff.calculate(Units.degreesToRadians(io.getCurrentAngleDegrees()), Units.degreesToRadians(io.getVelocityDegreesPerSecond()));

        effort += feedforward;
        effort = MathUtil.clamp(effort, -12, 12);

        io.setVoltage(effort);

        SmartDashboard.putNumber("MANIPULATOR FEED FORWARD", feedforward);
        SmartDashboard.putNumber("MANIPULATOR TARGET POSITION", targetPosition);
        SmartDashboard.putNumber("MANIPULATOR EFFORT", effort);
        SmartDashboard.putNumber("Manipulator Encoder Value", getPosition());
    }

    /**
     * Moves the manipulator to the desired position, using voltage.
     * 
     * @param target Desired manipulator position in degrees
     */
    public void setPosition(double target) {
        targetPosition = target;
    }
    
    /**
     * Gets the manipulator motor position in degrees
     */
    public double getPosition(){
        return io.getCurrentAngleDegrees();
    }

    public double getRollerVoltage() {
        return io.getRollerVoltage();
    }

    public void setRollerSpeed(double speed) {
        io.setRollerSpeed(speed);
    }

}
