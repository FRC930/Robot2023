package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final ProfiledPIDController controller;
    private final ArmFeedforward ff;

    private double targetPosition;

    private final ArmIO io;

    //TODO use offsets for positions
    public static double highPosition = 10; //at high elevator position

    public static double mediumPosition = 17.9; //at medium elevator position

    public static double groundPosition = 46.8; //at ground elevator position

    public static double stowPosition = 120.0;//-60.0; //at ground elevator position

    //TODO: These are nonsensical (Fix once we get actual values)
    public static double intakePosition = 50.0;

    public static double armLength = 27.12;

    /**
     * <h3>ArmSubsystem</h3>
     * 
     * Controls the motors and encoders, shoulder and wrist, on the arm.
     * 
     * @param io The ArmIO, use IORobot if robot is real, otherwise use IOSim.
     */
    public ArmSubsystem (ArmIO io) {

        // Sets up PID controller
        // controller = new ProfiledPIDController(0.2, 0, 0, new Constraints(50, 100));
        controller = new ProfiledPIDController(0.2, 0, 0, new Constraints(10, 10));
        controller.setTolerance(1, 1);
        controller.enableContinuousInput(0, 360);

        // TODO Change values when manipulator is added
        ff = new ArmFeedforward(0, 0.3, 0);

        this.io = io;

        targetPosition = 0;//stowPosition;
    }

    /**
     * <h3>periodic</h3>
     * 
     * Gets the inputs from the IO, and uses the feed forward and the PID controller to calculate the effort, in volts, to set to the io,
     * for both the shoulder and wrist motors.
     */
    @Override
    public void periodic() {
        this.io.updateInputs();
        // caculate PID and Feet forward angles 

        double effort = controller.calculate(io.getCurrentAngleDegrees(), targetPosition);
        double feedforward = ff.calculate(Units.degreesToRadians(io.getCurrentAngleDegrees()), Units.degreesToRadians(io.getVelocityDegreesPerSecond()));

        effort += feedforward;
        effort = MathUtil.clamp(effort, -6, 6);

        io.setVoltage(effort);

        SmartDashboard.putNumber("ARM FEED FORWARD", feedforward);
        SmartDashboard.putNumber("ARM TARGET POSITION", targetPosition);
        SmartDashboard.putNumber("ARM EFFORT", effort);
        //SmartDashboard.putNumber("ARM SETPOINT", controller.getSetpoint());
        SmartDashboard.putNumber("ARM ERROR", controller.getPositionError());
        SmartDashboard.putNumber("Arm Encoder Value", getPosition());
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
        return io.getCurrentAngleDegrees();
    }

}
