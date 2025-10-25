// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Claw;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Default_Claw extends Command {
  /** Creates a new Default_Claw. */
  private Claw s_Claw;

  public Default_Claw(Claw s_Claw) {
    this.s_Claw = s_Claw;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Claw);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Claw.setPosition(Constants.ClawConstants.default_pos);
    s_Claw.WristDown();
    s_Claw.Claw_Stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
