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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;
import frc.robot.subsystems.TurretManager;
import frc.robot.subsystems.TurretRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterRollers;
import frc.robot.subsystems.ShooterWristRev;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DefaultCommands.DefaultClimberCommand;
import frc.robot.commands.DefaultCommands.DefaultIndexCommand;
import frc.robot.commands.DefaultCommands.DefaultIntakeCommand;
import frc.robot.commands.DefaultCommands.DefaultShooterRollersCommand;
import frc.robot.commands.DefaultCommands.DefaultShooterWristCommand;
import frc.robot.commands.DefaultCommands.DefaultTurretCommand;
import frc.robot.commands.DriveCommands.DriveRobotCentric;
import frc.robot.commands.LimelightCommands.AimTurretAtHub;
import frc.robot.commands.LimelightCommands.LockOnAprilTag;
import frc.robot.commands.LimelightCommands.TrackObject;
import frc.robot.constants.CommandConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.state.IDLE;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexRollers;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWristRev;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
    private final ShooterRollers shootRollers = new ShooterRollers();
    private final IndexRollers indexRollers = new IndexRollers();
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final ShooterWristRev wrist = new ShooterWristRev();
    private final TurretRev turret = new TurretRev();
    private final IntakeWristRev intakeWrist = new IntakeWristRev();
    private final ClimberSubsystem climber = new ClimberSubsystem(stateManager);
    
    //limelights
    private final LimelightSubsystem turretLimelight = new LimelightSubsystem("limelight-fourtwo");
    private final LimelightSubsystem localLimelight = new LimelightSubsystem("limelight-four");

    
    private final LocalizationSubsystem localization = new LocalizationSubsystem(drivetrain, field, localLimelight, turretLimelight);
    
    private final TurretManager turretManager = new TurretManager(turret, wrist, localization, copilot);


    //factories
    private final CommandFactory commandFactory = new CommandFactory(intakeWrist, turret, wrist, turretLimelight, localLimelight, intakeRollers, indexRollers, shootRollers, climber, localization, pilot, drivetrain, stateManager);
    private final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(intakeWrist, turret, wrist, turretLimelight, localLimelight, intakeRollers, indexRollers, shootRollers, climber, localization, pilot, drivetrain, stateManager);
    
    
    private final SendableChooser<Command> chooser ;

    public RobotContainer() {
        stateManager.requestNewState(States.IDLE);
        chooser = AutoBuilder.buildAutoChooser("Autonomous");
        SmartDashboard.putData("Autos", chooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-pilot.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-pilot.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Default Commands
        shootRollers.setDefaultCommand(new DefaultShooterRollersCommand(shootRollers, stateManager));
        indexRollers.setDefaultCommand(new DefaultIndexCommand(indexRollers, stateManager));
        intakeRollers.setDefaultCommand(new DefaultIntakeCommand(intakeRollers, stateManager));
        wrist.setDefaultCommand(new DefaultShooterWristCommand(wrist, turretManager));
        turret.setDefaultCommand(new DefaultTurretCommand(turret, turretManager));
        climber.setDefaultCommand(new DefaultClimberCommand(climber, stateManager));

        configureMainBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureMainBindings() {
        // Drive Controls
        pilot.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        pilot.start().onTrue(new RequestStateChange(States.IDLE, stateManager));
        pilot.b().whileTrue(new DriveRobotCentric(drivetrain, pilot));

        pilot.x().onTrue(new RequestStateChange(States.CONDENSED, stateManager));

        pilot.b().onTrue(commandFactory.ClimbUpCommand());
        pilot.a().onTrue(commandFactory.ClimbDownCommand());

        pilot.rightTrigger().whileTrue(commandFactory.GroundIntakeCommand());
        pilot.rightTrigger().onFalse(new RequestStateChange(States.IDLE, stateManager));

        copilot.leftTrigger(.7).whileTrue(drivetrain.applyRequest(()->brake));

        // Subsystem Controls
        copilot.rightTrigger(.7).whileTrue(commandFactory.HubShootCommand());
        // copilot.y().toggleOnTrue(new AimTurretAtHub(drivetrain, pilot, localization));
        // copilot.y().toggleOnFalse(new returnToIdle(stateManager));
        copilot.y().onTrue(new InstantCommand(() -> turretManager.hubAimToggle()));
        copilot.back().onTrue(new InstantCommand(() -> turretManager.manualAimToggle()));

        // Limelight Controls
        
    }

    public void changePipeline() {
        localLimelight.setPipeline(1);
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
