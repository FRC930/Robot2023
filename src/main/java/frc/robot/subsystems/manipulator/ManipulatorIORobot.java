package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class ManipulatorIORobot implements ManipulatorIO { 

    // -------- DECLARATIONS --------\\
    private final CANSparkMax manipulator;
    private final CANSparkMax roller;
    private final AbsoluteEncoder manipulatorEncoder;

    // -------- CONSTANTS --------\\
    // Constant, in amps
    private final int STALL_LIMIT = 10;
    private final int FREE_LIMIT = 20;

    private static double manipulatorOffset = 322; // -45 needed to adjust to get angle back to where tested


    //----------Constructor---------\\
    public ManipulatorIORobot(int manipulatorMotorID, int manipulatorRollerMotorID) {
        manipulator = new CANSparkMax(manipulatorMotorID, MotorType.kBrushless);
        roller = new CANSparkMax(manipulatorRollerMotorID, MotorType.kBrushless);

        manipulator.restoreFactoryDefaults();
        manipulator.setIdleMode(IdleMode.kBrake);
        roller.restoreFactoryDefaults();
        roller.setIdleMode(IdleMode.kBrake);


        // TODO: Determine if this helps encoder position update faster
        manipulator.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        manipulator.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        // TODO: Was too low when tested
        //roller.setSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT);

        // Initializes Absolute Encoder from motors
        manipulatorEncoder = manipulator.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        // Sets position and velocity conversion factors so units are in degrees and degrees/second
        manipulatorEncoder.setPositionConversionFactor(360);
        manipulatorEncoder.setVelocityConversionFactor(60);

        manipulator.setInverted(true);

        manipulatorEncoder.setZeroOffset(manipulatorOffset);
    }
     /**
     * <h3>updateInputs</h3>
     * 
     * Left blank because it's only used in simulation
     */
    @Override
    public void updateInputs() {}

     /**
     * <h3>getCurrentAngleDegrees</h3>
     * returns manipulator position
     * @return manipulatorEncoder position
     */
    @Override
    public double getCurrentAngleDegrees() {
        return manipulatorEncoder.getPosition();
    }
     /**
     * <h3>getVelocityDegreesPerSecond</h3>
     * 
     * Returns velocity provided by the manipulator encoder 
     */
    @Override
    public double getVelocityDegreesPerSecond() {
        return manipulatorEncoder.getVelocity();
    }
     /**
     * <h3>setVoltage</h3>
     * 
     * Sets voltage of manipulator to predifined volts value
     * @param volts
     */
    @Override
    public void setVoltage(double volts) {
        manipulator.setVoltage(volts);
    }
     /**
     * <h3>getRollerVoltage</h3>
     * Returns the current voltage of the roller
     * @return roller.getBusVoltage()
     */
    @Override
    public double getRollerVoltage() {
        return roller.getBusVoltage();
    }
     /**
     * <h3>setRollerSpeed</h3>
     * Set the speed of the roller
     * @param speed
     */
    @Override
    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }
}
