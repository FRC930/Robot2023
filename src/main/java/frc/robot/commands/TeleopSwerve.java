package frc.robot.commands; 

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TeleopSwerve extends CommandBase {

    // ----- CONSTANTS -----\\

    private double stickDeadband = 0.1;
    // TODO Increase priority on this process (trick from example robot)
    public static final double percentSpeed = 0.5; // TODO increase for driver

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveDrive s_Swerve;
    private CommandXboxController controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(SwerveDrive s_Swerve, CommandXboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = -controller.getHID().getRawAxis(translationAxis);
        double xAxis = -controller.getHID().getRawAxis(strafeAxis);
        double rAxis = -controller.getHID().getRawAxis(rotationAxis);
        
        /* Deadbands */
        yAxis = MathUtil.applyDeadband(yAxis, stickDeadband);
        xAxis = MathUtil.applyDeadband(xAxis, stickDeadband);
        rAxis = MathUtil.applyDeadband(rAxis, stickDeadband);

        double throttle = yAxis * percentSpeed;
        double strafe = xAxis * percentSpeed;
        rotation = rAxis * percentSpeed;

        s_Swerve.drive(throttle, strafe, rotation, fieldRelative, openLoop);
    }
}
