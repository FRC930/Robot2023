package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final ProfiledPIDController controller;
    private final ArmFeedforward ff;

    private CANSparkMax wristMotor;
    private CANSparkMax shoulderMotor; 

    private RelativeEncoder wristEncoder;
    private RelativeEncoder shoulderEncoder;


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
     * ArmSubsystem controls the motors and encoders, shoulder and wrist, on the arm.
     * 
     * @param WristMotorID ID of the wrist motor from the arm
     * @param ShoulderMotorID ID of the shoulder motor from the arm
     */
    public ArmSubsystem (int WristMotorID, int ShoulderMotorID) {

        // Sets up PID controller TODO: Change these values
        controller = new ProfiledPIDController(1, 0, 0, new Constraints(360, 360));
        controller.setTolerance(1, 1);

        // Sets up Feetforward TODO: Change these values
        ff = new ArmFeedforward(0, 0.1, 0);

        // Initializes motors
        wristMotor = new CANSparkMax(WristMotorID, MotorType.kBrushless);
        shoulderMotor = new CANSparkMax(ShoulderMotorID, MotorType.kBrushless);

        wristMotor.restoreFactoryDefaults();
        shoulderMotor.restoreFactoryDefaults(); 

        // Initializes AlternateEncoders from motors
        wristEncoder = wristMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        shoulderEncoder = shoulderMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        // Sets position and velocity conversion factors so units are in degrees and degrees/second
        wristEncoder.setPositionConversionFactor(360);
        wristEncoder.setVelocityConversionFactor(60);
        shoulderEncoder.setPositionConversionFactor(360);
        shoulderEncoder.setVelocityConversionFactor(60);
    }

    /**
     * Moves the wrist to the desired position, using voltage.
     * 
     * @param targetPosition Desired wrist position in degrees
     */
    public void setWristPosition(double targetPosition) {
        double effort = controller.calculate(getWristPosition(), targetPosition);
        double feedforward = ff.calculate(getWristPosition(), getWristVelocity());

        effort += feedforward;
        effort = MathUtil.clamp(effort, -12, 12);

        setWristVoltage(effort);
    }

    /**
     * Moves the shoulder to the desired position, using voltage.
     * 
     * @param targetPosition Desired shoulder position in degrees
     */
    public void setShoulderPosition(double targetPosition) {
        double effort = controller.calculate(getShoulderPosition(), targetPosition);
        double feedforward = ff.calculate(getShoulderPosition(), getShoulderVelocity());

        effort += feedforward;
        effort = MathUtil.clamp(effort, -12, 12);

        setShoulderVoltage(effort);
    }

    /**
     * Sets wrist motor to specified voltage.
     * 
     * @param volts Desired voltage to set to wrist motor.
     */
    public void setWristVoltage(double volts) {
        wristMotor.setVoltage(volts);
    }

    /**
     * Sets shoulder motor to specified voltage.
     * 
     * @param volts Desired voltage to set to shoulder motor.
     */
    public void setShoulderVoltage(double volts) {
        shoulderMotor.setVoltage(volts);
    }

    /**
     * Gets the current wrist position.
     * 
     * @return Returns the current position of the wrist in degrees.
     */
    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    /**
     * Gets the current shoulder position.
     * 
     * @return The current position of the shoulder in degrees.
     */
    public double getShoulderPosition() {
        return shoulderEncoder.getPosition();
    }

    /**
     * Gets the current velocity of the wrist motor.
     * 
     * @return The current velocity of the wrist in degrees per second.
     */
    public double getWristVelocity() {
        return wristEncoder.getVelocity();
    }

    /**
     * Gets the current velocity of the shoulder motor.
     * 
     * @return The current velocity of the shoulder in degrees per second.
     */
    public double getShoulderVelocity() {
        return shoulderEncoder.getVelocity();
    }

    /**
     * Gets the current voltage of the wrist motor.
     * 
     * @return The current voltage of the wrist in volts (Should be between -12 and 12).
     */
    public double getWristOutputCurrent() {
        return MathUtil.clamp(wristMotor.getOutputCurrent(), -12, 12);
    }

    /**
     * Gets the current voltage of the shoulder motor.
     * 
     * @return The current voltage of the shoulder in volts (Should be between -12 and 12).
     */
    public double getShoulderOutputCurrent() {
        return MathUtil.clamp(shoulderMotor.getOutputCurrent(), -12, 12);
    }


}
