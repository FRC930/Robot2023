package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    
    private final ElevatorIO io;
    private double targetElevatorPosition;

    private final ProfiledPIDController controller;
    private final ElevatorFeedforward ff;
   
    /**
     * <h3>ElevatorSubsystem</h3>
     * 
     * Controls the motors and encoders, shoulder and wrist, on the arm.
     * 
     * @param io The ElevatorIO, use IORobot if robot is real, otherwise use IOSim.
     */
    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
        this.controller = new ProfiledPIDController(72, 0, 0, 
                          new Constraints(1.0, 2.0)); //This is in meters
        this.ff = new ElevatorFeedforward(0, 0, 0, 0);
    }
    
    /**
     * <h3>setTargetElevatorPosition</h3>
     * 
     * Sets the Target Elevator Position in inches.
     */
    public void setTargetElevatorPosition(double inches){
        targetElevatorPosition = inches;
    }

    /**
     * <h3>getElevatorPosition</h3>
     * 
     * Gets where the elevator is in inches.
     */
    public double getElevatorPosition(){
        return io.getCurrentHeight();
    }

    /**
    * Gets the inputs from the IO, and uses the feed forward and the PID controller to calculate the effort, in volts, to set to the io,
    * for the elevator motor.
     */
    @Override
    public void periodic() {
        io.updateInputs();

        double voltage = controller.calculate(io.getCurrentHeight(), targetElevatorPosition);
        double feedforward = ff.calculate(io.getCurrentVelocity());
        MathUtil.clamp(voltage, -12, 12);

        io.setVoltage(voltage + feedforward);

        //Updates shuffleboard values for elevator
        SmartDashboard.putNumber("ELEVATOR TARGET POSITION", targetElevatorPosition);
        SmartDashboard.putNumber("Elevator Encoder Value: ", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Encoder Value (Inches): ", Units.metersToInches(getElevatorPosition()));
        SmartDashboard.putNumber("ELEVATOR VOLTAGE", voltage + feedforward);
    }
}
