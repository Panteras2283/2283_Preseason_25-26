// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Ground_algae extends Command {
  private Claw s_Claw;
  private Elevator s_Elevator;
  private LEDs s_LEDs;
  /** Creates a new Net. */
  public Ground_algae(Claw s_Claw, Elevator s_Elevator, LEDs s_LEDs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Claw = s_Claw;
    this.s_Elevator = s_Elevator;
    this.s_LEDs = s_LEDs;

    addRequirements(s_Claw, s_Elevator, s_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        s_Elevator.setPosition(Constants.ElevatorConstants.leftFloorAlgae_pos, Constants.ElevatorConstants.rightFloorAlgae_pos);
        if (s_Elevator.getLeftPos() >= Constants.ElevatorConstants.leftFloorAlgae_pos - 0.5 || s_Elevator.getRightPos() <= Constants.ElevatorConstants.rightFloorAlgae_pos + 0.5){
          s_Claw.setPosition(Constants.ClawConstants.groundAlgae_pos);
          s_Claw.WristUp();
          s_Claw.Claw_Grab();
          s_LEDs.Algae();
        }

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

