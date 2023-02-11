package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public class ElevatorIORobot implements ElevatorIO {
    private CANSparkMax rightElevatorMaster;
    private RelativeEncoder rightElevatorEncoder;
    double circumference = Units.inchesToMeters(1.128) * Math.PI; //1.128 is diameter of pulley
    double gearRatio = 4.0; //4.0 is the gear ratio of motor to belt

    public ElevatorIORobot(int rightMotorID){//int leftMotorID, int rightMotorID){

        //LeftElevatorFollower = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        rightElevatorMaster = new CANSparkMax(rightMotorID, MotorType.kBrushless);

        rightElevatorEncoder = rightElevatorMaster.getEncoder();
    
        //LeftElevatorFollower.restoreFactoryDefaults();
        rightElevatorMaster.restoreFactoryDefaults();

        rightElevatorEncoder.setPositionConversionFactor(circumference / gearRatio);
        //LeftElevatorFollower.follow(RightElevatorMaster, true);
    }

    /**
     * <h3>updateInputs</h3>
     * 
     * Left blank because it's only used in simulation
     */
    @Override
    public void updateInputs() {}

    /**
     * <h3>getCurrentHeight</h3>
     * 
     * Gets the elevator motor position in meters
     * @return the elevator motor position
     */
    @Override
    public double getCurrentHeight() {
        return rightElevatorEncoder.getPosition();
    }

    /**
     * <h3>getCurrentVelocity</h3>
     * 
     * Gets the elevator motor's velocity
     * @return velocity of motor
     */
    @Override
    public double getCurrentVelocity() {
        return rightElevatorEncoder.getVelocity();
    }

    /**
     * <h3>setVoltage</h3>
     * 
     * Set the elevator motor voltage 
     * @param volts
     */
    @Override
    public void setVoltage(double volts) {
        rightElevatorMaster.setVoltage(volts);
    }
}
