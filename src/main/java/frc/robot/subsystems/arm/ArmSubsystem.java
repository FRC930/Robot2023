package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmIO.ArmInputs;

public class ArmSubsystem extends SubsystemBase {

    private final ArmIO io;
    private final ProfiledPIDController controller;
    private final ArmFeedforward ff;
    private ArmInputs inputs;

    private double targetPosition;

    //TODO: These are nonsensical (Fix once we get actual values)
    public static double highWristPosition = 27.6; //at high elevator position
    public static double highArmPosition = 10; //at high elevator position

    public static double mediumWristPosition = 23.9; //at medium elevator position
    public static double mediumArmPosition = 17.9; //at medium elevator position

    public static double groundWristPosition = 5.2; //at ground elevator position
    public static double groundArmPosition = 46.8; //at ground elevator position

    public static double stowWristPosition = 90.0; //at ground elevator position
    public static double stowArmPosition = -60.0; //at ground elevator position

    public static double intakeWristPosition = -225.0;
    public static double intakeArmPosition = 90.0;

    public ArmSubsystem (ArmIO io) {
        this.io = io;
        this.controller = new ProfiledPIDController(1, 0, 0, new Constraints(360, 360));
        this.controller.setTolerance(1, 1);
        this.ff = new ArmFeedforward(0, 0.1, 0);

        this.inputs = this.io.updateInputs();
    }

    @Override
    public void periodic() {
        this.inputs = this.io.updateInputs();
        double effort = controller.calculate(this.inputs.currentAngleDegrees(), targetPosition);
        double feedforward = this.ff.calculate(this.inputs.currentAngleDegrees(), this.inputs.currentVelocityDegreesPerSecond());

        effort += feedforward;
        effort = MathUtil.clamp(effort, -12, 12);

        SmartDashboard.putNumber("Arm Target Angle", this.targetPosition);

        this.io.setVoltage(effort);
    }

    private void setAngleDegrees(double angle) {
        SmartDashboard.putNumber("Arm Target Angle", angle);
        this.targetPosition = angle;
    }

    public boolean isAtSetpoint() {
        // ProfiledPIDController atGoal/atSetpoint is bugged and will never return true. Manually doing the same calculation minus the bugged property. Fixed in an upcoming release
        return this.controller.getPositionTolerance() > Math.abs(this.controller.getPositionError()) && this.controller.getGoal().equals(this.controller.getSetpoint());
    }

    public double getAngleDegrees() {
        this.inputs = this.io.updateInputs();
        return this.inputs.currentAngleDegrees();
    }

    public InstantCommand setAngleDeg(double targetDegrees) {
        return new InstantCommand(() -> setAngleDegrees(targetDegrees), this);
    }

    public Command waitForStable() {
        return new WaitUntilCommand(() -> isAtSetpoint());
    }

    public SequentialCommandGroup pointToPoint() {
        return new SequentialCommandGroup(
          setAngleDeg(45),
          waitForStable(),
          new WaitCommand(.5),
          setAngleDeg(90),
          waitForStable(),
          new WaitCommand(.5),
          setAngleDeg(135),
          waitForStable(),
          new WaitCommand(.5),
          setAngleDeg(180),
          waitForStable(),
          new WaitCommand(.5),
          setAngleDeg(215),
          waitForStable(),
          new WaitCommand(.5),
          setAngleDeg(180),
          waitForStable(),
          new WaitCommand(.5),
          setAngleDeg(135),
          waitForStable(),
          new WaitCommand(.5),
          setAngleDeg(90),
          waitForStable(),
          new WaitCommand(.5),
          setAngleDeg(45),
          waitForStable(),
          new WaitCommand(.5),
          setAngleDeg(0)
        );
    }

}
