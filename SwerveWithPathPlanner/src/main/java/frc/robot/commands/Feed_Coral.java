// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.Default_Claw;
import frc.robot.commands.Default_Elevator;
import edu.wpi.first.wpilibj2.command.Command;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Feed_Coral extends Command {
  private Claw s_Claw;
  private Elevator s_Elevator;
  private LEDs s_LEDs;
  /** Creates a new Test1. */
  
  public Feed_Coral(Claw s_Claw, Elevator s_Elevator, LEDs s_LEDs) {
    this.s_Claw = s_Claw;
    this.s_Elevator = s_Elevator;
    this.s_LEDs = s_LEDs;
    addRequirements(s_Claw, s_Elevator, s_LEDs);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Elevator.setPosition(Constants.ElevatorConstants.leftFS_pos, Constants.ElevatorConstants.rightFS_pos);
    if (s_Elevator.getLeftPos() >= Constants.ElevatorConstants.leftFS_pos - 0.5 || s_Elevator.getRightPos() <= Constants.ElevatorConstants.rightFS_pos + 0.5){
      s_Claw.setPosition(Constants.ClawConstants.feedCoral_pos);
      s_Claw.WristDown();
      s_Claw.Claw_FS();
      s_LEDs.Feeding();
    }

    if (s_Claw.getUpperSensor() == false || s_Claw.getLowerSensor() == false){
      s_Claw.Claw_Stop();
      s_Elevator.setPosition(Constants.ElevatorConstants.leftDefault_pos, Constants.ElevatorConstants.rightDefault_pos);
      s_Claw.setPosition(Constants.ClawConstants.default_pos);
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_Claw.getUpperSensor() == false || s_Claw.getLowerSensor() == false){
      s_LEDs.Coral();
      return true;
    } else {
      return false;
    }
  }
}
