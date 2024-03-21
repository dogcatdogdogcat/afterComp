package frc.robot;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.lib.Controller;
import frc.robot.commands.swerve.BrakeMode;
import frc.robot.commands.swerve.Drive;
import frc.robot.commands.swerve.TrackAprilTags;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Alerts;
import frc.robot.util.Constants;

import frc.robot.subsystems.ExampleSubsystem;

public class RobotContainer {
    

    private Swerve swerve;
    private Limelight limelight;
    private Controller driverOne;
    private final SendableChooser<Command> autoChooser;
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

    public RobotContainer() {

        NamedCommands.registerCommand("marker1", Commands.print("Passed Marker 1"));
        NamedCommands.registerCommand("ExampleCommand", exampleSubsystem.exampleMethodCommand());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", autoChooser);

        try { this.swerve = new Swerve(); } 
        catch (IOException ioException) { Alerts.swerveInitialized.set(true); }

        this.limelight = new Limelight();
        this.driverOne = new Controller(0);

        this.configureCommands();
    }

    public Command getAutonomousCommand(){
        PathPlannerPath linePath = PathPlannerPath.fromPathFile("Line Path");

        return AutoBuilder.followPath(linePath);
    }

    private void configureBindings(){
        SmartDashboard.putData("Line Auto", new PathPlannerAuto("Line Auto"));
    }

    private void configureCommands () {
       
            this.swerve.setDefaultCommand(new Drive(
                this.swerve, 
                () -> MathUtil.applyDeadband(-this.driverOne.getHID().getLeftY(), Constants.SwerveConstants.TRANSLATION_DEADBAND),
                () -> MathUtil.applyDeadband(-this.driverOne.getHID().getLeftX(), Constants.SwerveConstants.TRANSLATION_DEADBAND), 
                () -> MathUtil.applyDeadband(this.driverOne.getHID().getRightX(), Constants.SwerveConstants.OMEGA_DEADBAND), 
                () -> this.driverOne.getHID().getPOV()
        ));
        
        

        this.limelight.setDefaultCommand(new TrackAprilTags(this.swerve, this.limelight));

        this.driverOne.x().onTrue(new InstantCommand(this.swerve::zeroGyro, this.swerve));
        this.driverOne.leftBumper().whileTrue(new RepeatCommand(new InstantCommand(this.swerve::lock, this.swerve)));

        /**
        this.driverOne.leftTrigger().whileTrue(this.swerve.getDriveSysidRoutine());
        this.driverOne.rightTrigger().whileTrue(this.swerve.getAngleSysidRoutine());
        */
    }

    public void setBrakeMode (boolean brake) { new BrakeMode(this.swerve, brake).schedule(); }
}
