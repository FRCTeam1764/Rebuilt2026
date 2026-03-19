// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.CopyOnWriteArrayList;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;
import frc.robot.subsystems.TurretRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RollersSubsystem;
import frc.robot.subsystems.ShooterWristRev;
import frc.robot.commands.BasicCommands.ClimberCommandSpec;
import frc.robot.commands.BasicCommands.IntakeWristCommand;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.BasicCommands.ShooterWristCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DefaultCommands.DefaultClimberCommand;
import frc.robot.commands.DefaultCommands.DefaultIntakeWristCommand;
import frc.robot.commands.DefaultCommands.DefaultRollersCommand;
import frc.robot.commands.DefaultCommands.DefaultShooterWristCommand;
import frc.robot.commands.DefaultCommands.DefaultTurretCommand;
import frc.robot.commands.DriveCommands.DriveRobotCentric;
import frc.robot.commands.LimelightCommands.LockOnAprilTag;
import frc.robot.commands.LimelightCommands.TrackObject;
import frc.robot.constants.CommandConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.state.IDLE;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeWristRev;

public class RobotContainer {
    private double MaxSpeed = 0.8 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 0.8*RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.06).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.025).withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final StateManager stateManager = new StateManager();

    
    private final Field2d field = new Field2d();
    
    //subsystems
    private final RollersSubsystem rollers = new RollersSubsystem();
    private final ShooterWristRev shooterWrist = new ShooterWristRev();
    private final TurretRev turret = new TurretRev();
    private final IntakeWristRev intakeWrist = new IntakeWristRev();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    
    //limelights
    private final LimelightSubsystem turretLimelight = new LimelightSubsystem("limelight-fourtwo");
    //private final LimelightSubsystem localLimelight = new LimelightSubsystem("limelight-four");

    
    //private final LocalizationSubsystem localization = new LocalizationSubsystem(drivetrain, field, localLimelight, turretLimelight);
    

    //factories
    private final CommandFactory commandFactory = new CommandFactory(intakeWrist, turret, shooterWrist, turretLimelight, rollers, climber, pilot, drivetrain, stateManager);
    private final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(intakeWrist, turret, shooterWrist, turretLimelight, rollers, climber, pilot, drivetrain, stateManager);
    
    
    //private final SendableChooser<Command> chooser;

    public RobotContainer() {
        drivetrain.configureAutoBuilder();
        stateManager.requestNewState(States.IDLE);
        // chooser = AutoBuilder.buildAutoChooser("Autonomous");
        // SmartDashboard.putData("Autos", chooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Math.pow(pilot.getLeftY(), 2) * (pilot.getLeftY()<0 ? -1: 1)  * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-pilot.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Default Commands
        rollers.setDefaultCommand(new DefaultRollersCommand(rollers, stateManager));
        intakeWrist.setDefaultCommand(new DefaultIntakeWristCommand(intakeWrist, stateManager));
        turret.setDefaultCommand(new DefaultTurretCommand(turret, copilot));
        shooterWrist.setDefaultCommand(new DefaultShooterWristCommand(shooterWrist, copilot));
        
        configureMainBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureMainBindings() {
        // Drive Controls
        pilot.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        //copilot.leftTrigger(.7).whileTrue(drivetrain.applyRequest(()->brake));
        
        pilot.start().onFalse(new RequestStateChange(States.IDLE, stateManager));


        // Subsystem Controls
        //pilot.b().whileTrue(new RequestStateChange(States.MID_IDLE, stateManager));


        // pilot.b().onTrue(commandFactory.ClimbUpCommand());
        // pilot.a().onTrue(commandFactory.ClimbDownCommand());

        pilot.rightTrigger().onTrue(commandFactory.GroundIntakeCommand());
        pilot.rightTrigger().onFalse(new RequestStateChange(States.IDLE, stateManager));

        pilot.leftBumper().onTrue(new RequestStateChange(States.INTAKE_WHILE_SHOOT, stateManager));
        pilot.leftBumper().onFalse(new RequestStateChange(States.IDLE, stateManager));

        pilot.rightBumper().onTrue(new RequestStateChange(States.CONDENSED, stateManager));
        pilot.rightBumper().onFalse(new RequestStateChange(States.IDLE, stateManager));
        
        copilot.rightTrigger().onTrue(commandFactory.ShootRampCommand());
        copilot.rightTrigger().onFalse(new RequestStateChange(States.IDLE, stateManager));

        // copilot.a().whileTrue(new ShooterWristCommand(CommandConstants.SHOOTER_FAR, shooterWrist));
        
        // copilot.x().whileTrue(new ShooterWristCommand(CommandConstants.SHOOTER_MID1, shooterWrist));
        
        // copilot.b().whileTrue(new ShooterWristCommand(CommandConstants.SHOOTER_MID2, shooterWrist));
       
        // copilot.y().whileTrue(new ShooterWristCommand(CommandConstants.SHOOTER_CLOSE, shooterWrist));
       
        copilot.leftBumper().onTrue(new RequestStateChange(States.INTAKE_WHILE_SHOOT, stateManager));
        copilot.leftBumper().onFalse(new RequestStateChange(States.IDLE, stateManager));

        copilot.leftTrigger().onTrue(new RequestStateChange(States.SHOOT_WITH_INTAKE, stateManager));
        copilot.leftTrigger().onFalse(new RequestStateChange(States.IDLE, stateManager));

        // copilot.pov(0).whileTrue(commandFactory.ClimbUpCommand());
        // copilot.pov(0).onFalse(new ClimberCommandSpec(0, climber));
        // copilot.pov(180).whileTrue(commandFactory.ClimbDownCommand());
        // copilot.pov(180).onFalse(new ClimberCommandSpec(0, climber));

    

        pilot.pov(90).whileTrue(new RequestStateChange(States.INTAKE_OUT, stateManager));


        // Aiming Controls
        // copilot.pov(0).whileTrue(new ShooterWristCommand(10, wrist));
        // copilot.pov(90).whileTrue(new ShooterWristCommand(15, wrist));
        // copilot.pov(180).whileTrue(new ShooterWristCommand(20, wrist));
        // copilot.pov(270).whileTrue(new ShooterWristCommand(30, wrist));
        
    }

    // public Command getAutonomousCommand() {
    //     return chooser.getSelected();
    // }
}
