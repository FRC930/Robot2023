package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class ElevatorIORobot implements ElevatorIO {
    private CANSparkMax rightElevatorMaster;
    private RelativeEncoder rightElevatorEncoder;

    public ElevatorIORobot(int rightMotorID){//int leftMotorID, int rightMotorID){

        //LeftElevatorFollower = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        rightElevatorMaster = new CANSparkMax(rightMotorID, MotorType.kBrushless);

        rightElevatorEncoder = rightElevatorMaster.getEncoder();
    
        //LeftElevatorFollower.restoreFactoryDefaults();
        rightElevatorMaster.restoreFactoryDefaults();

        //LeftElevatorFollower.follow(RightElevatorMaster, true);
    }

    @Override
    public void updateInputs() {}

    @Override
    public double getOutputVoltage() {
        return MathUtil.clamp(rightElevatorMaster.getOutputCurrent(), -12, 12);
    }

    @Override
    public double getCurrentHeight() {
        return rightElevatorEncoder.getPosition();
    }

    @Override
    public double getCurrentVelocity() {
        return rightElevatorEncoder.getVelocity();
    }

    @Override
    public void setVoltage(double volts) {
        rightElevatorMaster.setVoltage(volts);
    }
    
}
