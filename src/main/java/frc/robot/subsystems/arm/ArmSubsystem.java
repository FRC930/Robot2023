package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final ProfiledPIDController wristController;
    private final ArmFeedforward wristff;
    private final ProfiledPIDController armController;
    private final ArmFeedforward armff;

    private double targetWristPosition;
    private double targetShoulderPosition;

    private final ArmIO io;

    public static double highWristPosition = 27.6; //at high elevator position
    public static double highShoulderPosition = 10; //at high elevator position

    public static double mediumWristPosition = 23.9; //at medium elevator position
    public static double mediumShoulderPosition = 17.9; //at medium elevator position

    public static double groundWristPosition = 5.2; //at ground elevator position
    public static double groundShoulderPosition = 46.8; //at ground elevator position

    public static double stowWristPosition = 90.0; //at ground elevator position
    public static double stowShoulderPosition = -60.0; //at ground elevator position

    //TODO: These are nonsensical (Fix once we get actual values)
    public static double intakeWristPosition = -225.0;
    public static double intakeShoulderPosition = 90.0;

    /**
     * Controls the motors and encoders, shoulder and wrist, on the arm.
     * 
     * @param io The ArmIO, use IORobot if robot is real, otherwise use IOSim.
     */
    public ArmSubsystem (ArmIO io) {

        // Sets up PID controller TODO: Change these values
        wristController = new ProfiledPIDController(1, 0, 0, new Constraints(360, 360));
        wristController.setTolerance(1, 1);
        armController = new ProfiledPIDController(1, 0, 0, new Constraints(360, 360));
        armController.setTolerance(1, 1);

        // Sets up Feetforward TODO: Change these values
        wristff = new ArmFeedforward(0, 0.1, 0);
        armff = new ArmFeedforward(0, 0.1, 0);

        this.io = io;

        targetWristPosition = stowWristPosition;
        targetShoulderPosition = stowShoulderPosition;
    }

    /**
    * Gets the inputs from the IO, and uses the feed forward and the PID controller to calculate the effort, in volts, to set to the io,
    * for both the shoulder and wrist motors.
     */
    @Override
    public void periodic() {
        this.io.updateInputs();
        // caculate PID and Feet forward angles 
        double wristEffort = wristController.calculate(io.getWristCurrentAngleDegrees(), targetWristPosition);
        double wristFeedforward = wristff.calculate(io.getWristCurrentAngleDegrees(), io.getWristVelocityDegreesPerSecond());

        wristEffort += wristFeedforward;
        wristEffort = MathUtil.clamp(wristEffort, -12, 12);

        io.setWristVoltage(wristEffort);


        double shoulderEffort = armController.calculate(io.getShoulderCurrentAngleDegrees(), targetShoulderPosition);
        double shoulderFeedforward = armff.calculate(io.getShoulderCurrentAngleDegrees(), io.getShoulderVelocityDegreesPerSecond());

        shoulderEffort += shoulderFeedforward;
        shoulderEffort = MathUtil.clamp(shoulderEffort, -12, 12);

        io.setShoulderVoltage(shoulderEffort);
    }

    /**
     * Moves the wrist to the desired position, using voltage.
     * 
     * @param targetPosition Desired wrist position in degrees
     */
    public void setWristPosition(double targetPosition) {
        targetWristPosition = targetPosition;
    }
    
    /**
     * Gets the Wrist motor position in degrees
     */
    public double getWristPosition(){
        return io.getWristCurrentAngleDegrees();
    }

    /**
     * Moves the shoulder to the desired position, using voltage.
     * 
     * @param targetPosition Desired shoulder position in degrees
     */
    public void setShoulderPosition(double targetPosition) {
        targetShoulderPosition = targetPosition;
    }

      /**
     * Gets the Shoulder motor position in degrees
     */
    public double getShoulderPosition(){
        return io.getShoulderCurrentAngleDegrees();
    }

}
