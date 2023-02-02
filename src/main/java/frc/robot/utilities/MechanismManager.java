package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmSubsystem;

public class MechanismManager {
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d m_elevator;
  private final MechanismLigament2d m_four;

  private final ArmSubsystem arm;
  // private final MechanismLigament2d m_four_two;

  public MechanismManager(ArmSubsystem arm) {
    this.arm = arm;
    mech = new Mechanism2d(50, 50);

    // Root node
    root = mech.getRoot("climber", 20, 20);

    // Subsystems

    // Elevator
    m_elevator =
        root.append(
            new MechanismLigament2d(
                "Elevator",
                0,
                0,
                30,
                new Color8Bit(Color.kGreen)));

    m_four =
        m_elevator.append(
            new MechanismLigament2d(
                "Four", 12.657, arm.getAngleDegrees(), 6, new Color8Bit(Color.kYellow)));

    // Log to SmartDashboard
    SmartDashboard.putData("Mech2d", mech);
  }

  public void periodic() {
    m_four.setAngle(arm.getAngleDegrees());
    // m_four_two.setAngle(Units.radiansToDegrees(Robot.four.getCurrentRads()));
    m_elevator.setLength(0);
  }
}