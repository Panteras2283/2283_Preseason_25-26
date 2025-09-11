// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


public class Elevator extends SubsystemBase {

  private SparkMax elevatorLeft = new SparkMax(Constants.ElevatorConstants.elevatorLeftID, MotorType.kBrushless);
  private SparkMax elevatorRight = new SparkMax(Constants.ElevatorConstants.elevatorRightID, MotorType.kBrushless);
  private final DigitalInput SensorUpper = new DigitalInput(Constants.ClawConstants.sensorUpperID);
  private final DigitalInput SensorLower = new DigitalInput(Constants.ClawConstants.sensorLowerID);

  private SparkMaxConfig leftElevatorConfig = new SparkMaxConfig();
  private SparkClosedLoopController leftElevatorController = elevatorLeft.getClosedLoopController();
  private RelativeEncoder leftElevatorEncoder = elevatorLeft.getEncoder();


  private SparkMaxConfig rightElevatorConfig = new SparkMaxConfig();
  private SparkClosedLoopController rightElevatorController = elevatorRight.getClosedLoopController();
  private RelativeEncoder rightElevatorEncoder = elevatorRight.getEncoder();


  public Elevator() {

    //LEFT ELEVATOR CONFIG
    leftElevatorConfig.closedLoop.p(Constants.ElevatorConstants.elevatorKP);
    leftElevatorConfig.closedLoop.i(Constants.ElevatorConstants.elevatorKI);
    leftElevatorConfig.closedLoop.d(Constants.ElevatorConstants.elevatorKD);
    leftElevatorConfig.closedLoop.outputRange(Constants.ElevatorConstants.KMinOutput, Constants.ElevatorConstants.KMaxOutput);
    leftElevatorConfig.closedLoop.velocityFF(Constants.ElevatorConstants.elevatorKFF);
    leftElevatorConfig.closedLoop.maxMotion.maxVelocity(Constants.ElevatorConstants.maxVel);
    leftElevatorConfig.closedLoop.maxMotion.maxAcceleration(Constants.ElevatorConstants.maxAcc);
    leftElevatorConfig.closedLoop.maxMotion.allowedClosedLoopError(Constants.ElevatorConstants.allErr);
    leftElevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    leftElevatorConfig.encoder.positionConversionFactor(1);
    elevatorLeft.configure(leftElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    //RIGHT ELEVATOR CONFIG
    rightElevatorConfig.closedLoop.p(Constants.ElevatorConstants.elevatorKP);
    rightElevatorConfig.closedLoop.i(Constants.ElevatorConstants.elevatorKI);
    rightElevatorConfig.closedLoop.d(Constants.ElevatorConstants.elevatorKD);
    rightElevatorConfig.closedLoop.outputRange(Constants.ElevatorConstants.KMinOutput, Constants.ElevatorConstants.KMaxOutput);
    rightElevatorConfig.closedLoop.velocityFF(Constants.ElevatorConstants.elevatorKFF);
    rightElevatorConfig.closedLoop.maxMotion.maxVelocity(Constants.ElevatorConstants.maxVel);
    rightElevatorConfig.closedLoop.maxMotion.maxAcceleration(Constants.ElevatorConstants.maxAcc);
    rightElevatorConfig.closedLoop.maxMotion.allowedClosedLoopError(Constants.ElevatorConstants.allErr);
    rightElevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    rightElevatorConfig.encoder.positionConversionFactor(1);
    elevatorRight.configure(rightElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Position", leftElevatorEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", rightElevatorEncoder.getPosition());
    SmartDashboard.putNumber("Left Velocity", leftElevatorEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", rightElevatorEncoder.getVelocity());
    SmartDashboard.putNumber("Left Output", elevatorLeft.getAppliedOutput());
    SmartDashboard.putNumber("Right Output", elevatorRight.getAppliedOutput());
    SmartDashboard.putBoolean("Upper Sensor", SensorUpper.get());
    SmartDashboard.putBoolean("Lower Sensor", SensorLower.get());

  }

  public void setPosition(double left_pos, double right_pos){
    leftElevatorController.setReference(left_pos, ControlType.kPosition);
    rightElevatorController.setReference(right_pos, ControlType.kPosition);
  }

  public double getLeftPos(){
    return leftElevatorEncoder.getPosition();
  }

  public double getRightPos(){
    return rightElevatorEncoder.getPosition();
  }

  public void resetEncoders(){
    leftElevatorEncoder.setPosition(0);
    rightElevatorEncoder.setPosition(0);
  }

  public void fullDown(){
    elevatorLeft.set(-0.1);
    elevatorRight.set(-0.1);
    System.out.println("ELEVATOR DOWN");
  }

  public void fullStop(){
    elevatorLeft.set(0);
    elevatorRight.set(0);
  }
}
