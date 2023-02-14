package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class ManipulatorIORobot implements ManipulatorIO { 

    private final CANSparkMax manipulator;
    private final CANSparkMax roller;

    private final AbsoluteEncoder manipulatorEncoder;

    // Constant, in amps
    private final int STALL_LIMIT = 10;
    private final int FREE_LIMIT = 20;

    private static double manipulatorOffset = -153.47;

    public ManipulatorIORobot(int manipulatorMotorID, int manipulatorRollerMotorID) {
        manipulator = new CANSparkMax(manipulatorMotorID, MotorType.kBrushless);
        roller = new CANSparkMax(manipulatorRollerMotorID, MotorType.kBrushless);

        manipulator.restoreFactoryDefaults();
        roller.restoreFactoryDefaults();

        //roller.setSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT);

        // Initializes Absolute Encoder from motors
        manipulatorEncoder = manipulator.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        // Sets position and velocity conversion factors so units are in degrees and degrees/second
        manipulatorEncoder.setPositionConversionFactor(360);
        manipulatorEncoder.setVelocityConversionFactor(60);

        
    }

    @Override
    public void updateInputs() {}

    @Override
    public double getCurrentAngleDegrees() {
        return manipulatorEncoder.getPosition() + manipulatorOffset;
    }

    @Override
    public double getVelocityDegreesPerSecond() {
        return manipulatorEncoder.getVelocity();
    }

    @Override
    public void setVoltage(double volts) {
        manipulator.setVoltage(volts);
    }

    @Override
    public double getRollerVoltage() {
        return roller.getBusVoltage();
    }

    @Override
    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }
}
