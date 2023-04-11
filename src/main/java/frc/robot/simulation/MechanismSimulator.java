package frc.robot.simulation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.rotateintake.PitchIntakeSubsystem;

public class MechanismSimulator {
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d m_elevator;
    private final MechanismLigament2d m_arm;
    private final MechanismLigament2d m_hand;
    private final MechanismLigament2d m_intake = null;

    private final ArmSubsystem arm;
    private final ManipulatorSubsystem manipulator;
    private final ElevatorSubsystem elevator;
    private final PitchIntakeSubsystem intake = null;

    private final SwerveDrive swerve;

    private static double elevatorYOffset = 12.83;
    private static double elevatorZOffset = 12.07;
    private static double elevatorXOffset = 0;

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
    public MechanismSimulator(ArmSubsystem arm, ElevatorSubsystem elevator, ManipulatorSubsystem manipulator, /*  PitchIntakeSubsystem intake,*/ SwerveDrive swerve){
        this.arm = arm;
        this.elevator = elevator;
        this.manipulator = manipulator;
        //this.intake = intake;
        this.swerve = swerve;

        manipulatorYOffset = Math.cos(arm.getPosition()) * ArmSubsystem.ARM_LENGTH;
        manipulatorZOffset = Math.sin(arm.getPosition()) * ArmSubsystem.ARM_LENGTH;

        mech = new Mechanism2d(Units.inchesToMeters(100), Units.inchesToMeters(100));
        root = mech.getRoot("elevator", Units.inchesToMeters(7.35), Units.inchesToMeters(10));
        mech.getRoot("Robot", 0, Units.inchesToMeters(7)).append(
            new MechanismLigament2d("frame", Units.inchesToMeters(26+7.5), 0, 5, new Color8Bit(Color.kBlanchedAlmond))
        );
        
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
                Units.inchesToMeters(24.719),
                arm.getPosition(),
                6,
                new Color8Bit(Color.kYellow)
                )  
            );
        
        //Adds manipulator to the arm in simulation
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
        
        // Adds intake to the root robot simulation
        // m_intake =
        //     root.append(
        //         new MechanismLigament2d(
        //             "Intake",
        //             Units.inchesToMeters(9.4),
        //             this.intake.getEncoderPosition(),
        //             6,
        //             new Color8Bit(Color.kBlueViolet)
        //         )
        //     );
        
    }
    
    /**
     * <h3>periodic</h3>
     * 
     * Constantly updates current simulation angle to the display and update the positions for Advantage Scope
     */
    public void periodic(){
        m_elevator.setLength(elevator.getElevatorPosition());
        m_arm.setAngle(arm.getPosition()-45);
        m_hand.setAngle(manipulator.getPosition()-arm.getPosition()-45);
        //m_intake.setAngle(intake.getEncoderPosition());

        // Elevator Position in Advantage Scope
        double[] elevatorPosition = {
            elevatorXOffset + swerve.getPose().getX(),
            elevatorYOffset + elevator.getElevatorPosition() + swerve.getPose().getY(),
            elevatorZOffset + elevator.getElevatorPosition(),
            1,
            0,
            0,
            swerve.getHeadingDegrees()};
        Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/ElevatorPos", elevatorPosition);
        
        // Arm Position in Advantage Scope
        double[] armPosition = {
            armXOffset + swerve.getPose().getX(),
            armYOffset + swerve.getPose().getY() + elevator.getElevatorPosition(),
            armZOffset + elevator.getElevatorPosition(),
            1,
            0,
            arm.getPosition(),
            swerve.getHeadingDegrees()
        };
        Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/ArmPos", armPosition);

        // Manipulator Position in Advantage Scope
        double[] manipulatorPosition = {
            manipulatorXOffset + swerve.getPose().getX(),
            manipulatorYOffset + elevator.getElevatorPosition() + swerve.getPose().getY(),
            manipulatorZOffset + elevator.getElevatorPosition(),
            1,
            0,
            manipulator.getPosition()-arm.getPosition(),
            swerve.getHeadingDegrees()
        };
        Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/ManipulatorPos", manipulatorPosition);

        // Sends system simulations to logger
        Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/Mech2d", mech);
    }
}
