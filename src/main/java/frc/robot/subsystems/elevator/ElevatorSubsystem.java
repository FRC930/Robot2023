package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase{
    
    
    private final ElevatorIO m_io;
    private static final double UPPER_STAGE_HIGHT = 20.0;
    private double targetElevatorPosition = 0;

    private final ProfiledPIDController controller;
    private final ElevatorFeedforward ff;
    private final ElevatorFeedforward topff;
   
    /**
     * <h3>ElevatorSubsystem</h3>
     * 
     * Controls the motors and encoders, shoulder and wrist, on the arm.
     * 
     * @param io The ElevatorIO, use IORobot if robot is real, otherwise use IOSim.
     */
    public ElevatorSubsystem(ElevatorIO io){
        this.m_io = io;
        //this.controller = new ProfiledPIDController(72, 0, 0, 
        //new Constraints(1.0, 2.0)); //This is in meters
        //our p is in terms of meters, meaning you are multiplying a decmal by p
        //45
        this.controller = new ProfiledPIDController(45, 0.0, 0.0, 
                 new Constraints(Units.inchesToMeters(110.0), Units.inchesToMeters(90.0))); //This is in meters //110 175
        this.ff = new ElevatorFeedforward(0.0, 0.8, 0.0, 0.0);
        this.topff = new ElevatorFeedforward(0, 0.8, 0.0, 0.0);
        // TODO set tolerance
        this.controller.setTolerance(0.1, 0.1);
    }
    
    /**
     * <h3>setTargetElevatorPosition</h3>
     * 
     * Sets the Target Elevator Position in meters.
     */
    public void setTargetElevatorPosition(double meters){
        targetElevatorPosition = meters;
    }

    /**
     * <h3>getElevatorPosition</h3>
     * 
     * Gets where the elevator is in inches.
     */
    public double getElevatorPosition(){
        return m_io.getCurrentHeight();
    }

    /**
    * Gets the inputs from the IO, and uses the feed forward and the PID controller to calculate the effort, in volts, to set to the io,
    * for the elevator motor.
     */
    @Override
    public void periodic() {
        m_io.updateInputs();
        //Checks if the robot is a simulation
        if (DriverStation.isEnabled() || !Robot.isReal()) {
            double currentheight = m_io.getCurrentHeight();

            double voltage = controller.calculate(currentheight, targetElevatorPosition);
            double feedforward;
            if(currentheight < Units.inchesToMeters(UPPER_STAGE_HIGHT)) {
                feedforward  = ff.calculate(m_io.getCurrentVelocity());
            }else {
                feedforward  = topff.calculate(m_io.getCurrentVelocity());
            }

            double effort = voltage + feedforward;

            effort = MathUtil.clamp(effort, -12, 12);

            m_io.setVoltage(effort);

            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/Voltage", effort);
            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/Feedforward", feedforward);
        }
        
        //Updates shuffleboard values for elevator
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/TargetPosition", targetElevatorPosition);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/EncoderValue", getElevatorPosition());
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/EncoderValue(Inches)", Units.metersToInches(getElevatorPosition()));
    }
    private Boolean pastHeight(double meters) {
        double height = getElevatorPosition();
        System.out.println("height: " + height + " target: " + meters);
        return height > meters;
    }

    public Command waitUntilPastHeightCommand(double meters) {
        return Commands.waitUntil(() -> this.pastHeight(meters));
    }

    public Command setElevatorPositionCommand(double meters) {
        return new InstantCommand(() -> setTargetElevatorPosition(meters), this);
    }

    public Command createWaitUntilAtHeightCommand() {
        return Commands.waitUntil(() -> this.controller.atGoal());
    }
}
