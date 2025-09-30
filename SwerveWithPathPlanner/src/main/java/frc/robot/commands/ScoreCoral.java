// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.Default_Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
  private Elevator s_Elevator;
  private Claw s_Claw;
  private double left_pos;
  private double right_pos;
  private double claw_pos;
  /** Creates a new Test1. */
  
  public ScoreCoral(Elevator s_Elevator, Claw s_Claw, double left_pos, double right_pos, double claw_pos) {
    this.s_Elevator = s_Elevator;
    this.s_Claw = s_Claw;
    this.right_pos = right_pos;
    this.left_pos = left_pos;
    this.claw_pos = claw_pos;
    addRequirements(s_Elevator, s_Claw);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Elevator.setPosition(left_pos, right_pos);
    if(s_Elevator.getLeftPos() >= left_pos - 0.2 || s_Elevator.getRightPos() <= right_pos + 0.2){
      s_Claw.setPosition(claw_pos);
      if (s_Claw.getPosition() >= claw_pos -0.1 && s_Claw.getPosition() <= claw_pos + 0.1){
        if (left_pos == Constants.ElevatorConstants.leftL1_pos){
          s_Claw.Claw_ReleaseL1();
          if (s_Claw.getUpperSensor() == true || s_Claw.getLowerSensor() == true){
            s_Claw.Claw_Stop();
            s_Claw.setPosition(0);
            if (s_Claw.getPosition() == 0 || s_Claw.getPosition() < 1.5){
              s_Elevator.setPosition(Constants.ElevatorConstants.leftDefault_pos, Constants.ElevatorConstants.rightDefault_pos);
            }

          }

        } else {
          s_Claw.Claw_Release();
          if (s_Claw.getUpperSensor() == true || s_Claw.getLowerSensor() == true){
            s_Claw.Claw_Stop();
            s_Claw.setPosition(0);
            if (s_Claw.getPosition() == 0 || s_Claw.getPosition() < 0.5){
              s_Elevator.setPosition(Constants.ElevatorConstants.leftDefault_pos, Constants.ElevatorConstants.rightDefault_pos);
            }
          }
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Claw.Claw_Release();
    s_Claw.setPosition(Constants.ClawConstants.default_pos);
    s_Claw.WristDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((s_Claw.getUpperSensor() == true && s_Claw.getLowerSensor()==true) && (s_Elevator.getLeftPos() >= Constants.ElevatorConstants.leftDefault_pos - 0.05 || s_Elevator.getRightPos() <= Constants.ElevatorConstants.rightDefault_pos + 0.05)){
      return true;
    } else {
      return false;
    }
  }    
}
