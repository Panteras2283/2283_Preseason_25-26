// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;

import java.util.List;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
    

    //Pose2d targetPose = new Pose2d(16.21, 4.05, Rotation2d.fromDegrees(180));
    PathConstraints constraints = new PathConstraints(3, 2, Units.degreesToRadians(540), Units.degreesToRadians(720));

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.55).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(7);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(7);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(10);
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /*Subsystems */
   private final Claw s_Claw = new Claw();
   

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("AUTO 1");
        SmartDashboard.putData("Auto Mode", autoChooser);
        

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        s_Claw.setDefaultCommand(new Default_Claw(s_Claw));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-yLimiter.calculate(driver.getLeftY()*MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(-xLimiter.calculate(driver.getLeftX()*MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(-rotLimiter.calculate(driver.getRightX()*MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        driver.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       /*  driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

        // reset the field-centric heading on left bumper press
        driver.back().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.leftBumper().onTrue(new ProxyCommand(()-> drivetrain.PathfindToReefpath(0, constraints, 0.0)));
        driver.leftBumper().onFalse(drivetrain.getDefaultCommand());
        driver.y().onTrue(new ProxyCommand(()-> drivetrain.PathfindToReefpath(1, constraints, 0.0)));
        driver.y().onFalse(drivetrain.getDefaultCommand());

        driver.rightBumper().onTrue(new ProxyCommand(()-> drivetrain.PathfindToReefpath(2, constraints, 0.0)));
        driver.rightBumper().onFalse(drivetrain.getDefaultCommand());
        
        //driver.rightBumper().whileTrue(drivetrain.PathfindToPose(targetPose, constraints, 0.0));


        drivetrain.registerTelemetry(logger::telemeterize);

        //operator.button(7).onTrue(setPoseKey("FS1"));
        //operator.button(8).onTrue(setPoseKey("FS2"));
        //operator.button(9).onTrue(setPoseKey("PR"));
        operator.y().onTrue(new Test1(s_Claw));
        operator.y().onFalse(s_Claw.getDefaultCommand());
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
