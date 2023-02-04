package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
        controller = new ProfiledPIDController(1, 0, 0, new Constraints(360, 360));
        controller.setTolerance(1, 1);

        // Sets up Feetforward TODO: Change these values
        ff = new ArmFeedforward(0, 0.1, 0);

        this.io = io;

        targetPosition = stowPosition;
    }

    /**
    * Gets the inputs from the IO, and uses the feed forward and the PID controller to calculate the effort, in volts, to set to the io,
    * for the manipulator motor.
     */
    @Override
    public void periodic() {
        this.io.updateInputs();
        // caculate PID and Feet forward angles 
        double effort = controller.calculate(io.getCurrentAngleDegrees(), targetPosition);
        double feedforward = ff.calculate(io.getCurrentAngleDegrees(), io.getVelocityDegreesPerSecond());

        effort += feedforward;
        effort = MathUtil.clamp(effort, -12, 12);

        io.setVoltage(effort);
    }

    /**
     * Moves the manipulator to the desired position, using voltage.
     * 
     * @param target Desired manipulator position in degrees
     */
    public void setWristPosition(double target) {
        targetPosition = target;
    }
    
    /**
     * Gets the manipulator motor position in degrees
     */
    public double getPosition(){
        return io.getCurrentAngleDegrees();
    }

}
