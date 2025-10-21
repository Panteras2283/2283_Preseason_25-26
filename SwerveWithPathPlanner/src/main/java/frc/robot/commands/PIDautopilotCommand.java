// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDautopilotCommand extends Command {
  private final CommandSwerveDrivetrain s_Swerve;
  private final Pose2d targetPose;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;
  private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();
  /** Creates a new PIDautopilotCommand. */
  public PIDautopilotCommand(CommandSwerveDrivetrain s_Swerve, Pose2d targetPose) {
    this.s_Swerve = s_Swerve;
    this.targetPose = targetPose;

    xController = new PIDController(10.0, 0.0, 0.0); // Adjust gains as necessary
    yController = new PIDController(10.0, 0.0, 0.0); // Adjust gains as necessary
    rotationController = new PIDController(10.0, 0.0, 0.0); // Adjust gains as necessary
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("PIDautopilotCommand is running...");
    Pose2d currentPose = s_Swerve.getState().Pose;

    double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
    double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotationOutput = rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    xOutput = Math.max(-1, Math.min(1, xOutput));
    yOutput = Math.max(-1, Math.min(1, yOutput));
    rotationOutput = Math.max(-1, Math.min(1, rotationOutput));

    /*SwerveRequest request = new SwerveRequest.fromFieldRelativeSpeeds(
      xOutput,
      yOutput,
      rotationOutput,
      s_Swerve.getRotation()
    );*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
