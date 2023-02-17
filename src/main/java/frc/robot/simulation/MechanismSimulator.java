package frc.robot.simulation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.rotateintake.PitchIntakeSubsystem;
import frc.robot.utilities.LogUtil;

public class MechanismSimulator {
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d m_elevator;
    private final MechanismLigament2d m_arm;
    private final MechanismLigament2d m_hand;
    private final MechanismLigament2d m_intake;

    private final ArmSubsystem arm;
    private final ManipulatorSubsystem manipulator;
    private final ElevatorSubsystem elevator;
    private final PitchIntakeSubsystem intake;

    private final SwerveDrive swerve;

    private static double elevatorXOffset = Units.inchesToMeters(-4.070);
    private static double elevatorZOffset = Units.inchesToMeters(15.732);
    private static double elevatorYOffset = 0;

    private static double armYOffset = 0;
    private static double armZOffset = 0;
    private static double armXOffset = 0;

    private static double manipulatorYOffset;
    private static double manipulatorZOffset;
    private static double manipulatorXOffset = 0;

    


    /**
     * Simulates the arm and elevator systems in simulation, in a 2d window.
     * 
     * @param arm Subsystem for the arm.
     * @param elevator Subsystem for the elevator.
     */
    public MechanismSimulator(ArmSubsystem arm, ElevatorSubsystem elevator, ManipulatorSubsystem manipulator, PitchIntakeSubsystem intake, SwerveDrive swerve){
        this.arm = arm;
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.intake = intake;
        this.swerve = swerve;

        manipulatorYOffset = Math.cos(arm.getPosition()) * ArmSubsystem.armLength;
        manipulatorZOffset = Math.sin(arm.getPosition()) * ArmSubsystem.armLength;

        mech = new Mechanism2d(Units.inchesToMeters(100), Units.inchesToMeters(100));
        root = mech.getRoot("robot", Units.inchesToMeters(20), Units.inchesToMeters(20));
        

        
        // Adds elevator to the robot in simulation
        m_elevator =
            root.append(
                new MechanismLigament2d(
                    "Elevator",
                    Units.inchesToMeters(10),
                    45,
                    10,
                    new Color8Bit(Color.kGreen)
                )
            );
        
        // Adds arm to the elevator in simulation
        m_arm =
            m_elevator.append(
              new MechanismLigament2d(
                "Arm",
                Units.inchesToMeters(10),
                arm.getPosition(),
                6,
                new Color8Bit(Color.kYellow)
                )  
            );
        
        // Adds manipulator to the arm in simulation
        m_hand =
            m_arm.append(
                new MechanismLigament2d(
                    "Hand",
                    Units.inchesToMeters(5),
                    manipulator.getPosition(),
                    6,
                    new Color8Bit(Color.kRed)
                    )
            );
        
        
        m_intake =
            root.append(
                new MechanismLigament2d(
                    "Intake",
                    Units.inchesToMeters(10),
                    this.intake.getEncoderPosition(),
                    6,
                    new Color8Bit(Color.kBlueViolet)
                )
            );
        
        // Sends system simulations to the smart dashboard
        SmartDashboard.putData("Mech2d", mech);
    }
    
    /**
     * Constantly updates current simulation angle to the display
     */
    public void periodic(){
        m_elevator.setLength(elevator.getElevatorPosition());
        m_arm.setAngle(arm.getPosition());
        m_hand.setAngle(manipulator.getPosition()-arm.getPosition());
        m_intake.setAngle(intake.getEncoderPosition());

        Pose3d robot = new Pose3d(swerve.getPose());
        Pose3d carriage = robot.transformBy(
            new Transform3d(
                new Translation3d(
                    elevatorXOffset + (elevator.getElevatorPosition() / 2),
                    elevatorYOffset,
                    elevatorZOffset + (elevator.getElevatorPosition() / 2)
                ),
                new Rotation3d()
            )
        );

        // Advantage scope
        double[] elevatorPosition = {
            elevatorXOffset + swerve.getPose().getX(),
            elevatorYOffset + elevator.getElevatorPosition() + swerve.getPose().getY(),
            elevatorZOffset + elevator.getElevatorPosition(),
            1,
            0,
            0,
            swerve.getHeadingRotation2d().getRadians()};
        Logger.getInstance().recordOutput("Elevator Position Advantage Scope", LogUtil.toPoseArray3d(carriage));

        Pose3d armPose = carriage.transformBy(
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(8.821),
                    0,
                    0
                ),
                new Rotation3d(
                    0,
                    -Units.degreesToRadians(arm.getPosition()),
                    0
                )
            )
        );

        // double[] armPosition = {
        //     armXOffset + swerve.getPose().getX(),
        //     armYOffset + swerve.getPose().getY() + elevator.getElevatorPosition(),
        //     armZOffset + elevator.getElevatorPosition(),
        //     1,
        //     0,
        //     arm.getPosition(),
        //     swerve.getHeadingRotation2d().getDegrees()
        // };
        Logger.getInstance().recordOutput("Arm Position Advantage Scope", armPose);

        Pose3d wristPose = armPose.transformBy(
            new Transform3d(
                new Translation3d(

                ),
                new Rotation3d(

                )
            )
        );

        double[] manipulatorPosition = {
            manipulatorXOffset + swerve.getPose().getX(),
            manipulatorYOffset + elevator.getElevatorPosition() + swerve.getPose().getY(),
            manipulatorZOffset + elevator.getElevatorPosition(),
            1,
            0,
            manipulator.getPosition(),
            swerve.getHeadingRotation2d().getDegrees()
        };
        Logger.getInstance().recordOutput("Manipulator Position Advantage Scope", wristPose);
    }
}
