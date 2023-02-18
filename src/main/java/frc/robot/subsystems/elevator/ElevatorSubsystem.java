package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase{
    
    /**
     *
     */
    private static final double UPPER_STAGE_HIGHT = 20.0;
    private final ElevatorIO io;
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
        this.io = io;
        //this.controller = new ProfiledPIDController(72, 0, 0, 
                          //new Constraints(1.0, 2.0)); //This is in meters
                          //our p is in terms of meters, meaning you are multiplying a decmal by p
                          this.controller = new ProfiledPIDController(45, 0, 0, 
                          new Constraints(Units.inchesToMeters(36), Units.inchesToMeters(30))); //This is in meters
        //this.ff = new ElevatorFeedforward(0, 0, 0, 0);
        this.ff = new ElevatorFeedforward(0, 0.45, 0, 0);

        this.topff = new ElevatorFeedforward(0, 0.45, 0, 0);


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
        return io.getCurrentHeight();
    }

    /**
    * Gets the inputs from the IO, and uses the feed forward and the PID controller to calculate the effort, in volts, to set to the io,
    * for the elevator motor.
     */
    @Override
    public void periodic() {

        if (DriverStation.isEnabled() || !Robot.isReal()) {
            io.updateInputs();
            double currentheight = io.getCurrentHeight();

            double voltage = controller.calculate(currentheight, targetElevatorPosition);
            double feedforward;
            if(currentheight < Units.inchesToMeters(UPPER_STAGE_HIGHT)) {
                feedforward  = ff.calculate(io.getCurrentVelocity());
            }else {
                feedforward  = topff.calculate(io.getCurrentVelocity());
            }
            MathUtil.clamp(voltage, -12, 12);

            io.setVoltage(voltage + feedforward);

            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/Voltage", voltage + feedforward);
            SmartDashboard.putNumber(this.getClass().getSimpleName()+"/Feedforward", feedforward);
        }
        
        //Updates shuffleboard values for elevator
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/TargetPosition", targetElevatorPosition);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/EncoderValue", getElevatorPosition());
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/EncoderValue(Inches)", Units.metersToInches(getElevatorPosition()));
    }
}
