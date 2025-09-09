// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;



public class Claw extends SubsystemBase {
  private final DigitalInput SensorUpper = new DigitalInput(Constants.ClawConstants.sensorUpperID);
  private final DigitalInput SensorLower = new DigitalInput(Constants.ClawConstants.sensorLowerID);
  private SparkFlex clawElbow = new SparkFlex(Constants.ClawConstants.clawElbowID, MotorType.kBrushless);
  private TalonFX clawLeft = new TalonFX(Constants.ClawConstants.clawLeftID);
  private TalonFX clawRight = new TalonFX(Constants.ClawConstants.clawRightID);

  private VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  private SparkFlexConfig motorConfig = new SparkFlexConfig();
  private SparkClosedLoopController motorController = clawElbow.getClosedLoopController();
  private final RelativeEncoder motorEncoder = clawElbow.getEncoder();

  private final DoubleSolenoid wristSolenoid =
  new DoubleSolenoid(Constants.ClawConstants.PH_CAN_ID, 
  PneumaticsModuleType.CTREPCM, 
  Constants.ClawConstants.solenoidForwardChannel, 
  Constants.ClawConstants.solenoidReverseChannel);

  private final double gearRatio =  80.0/1.0;


  /** Creates a new Claw. */
  public Claw() {
    motorConfig.closedLoop
      .p(Constants.ClawConstants.clawKP)
      .i(Constants.ClawConstants.clawKI)
      .d(Constants.ClawConstants.clawKD)
     .outputRange(Constants.ClawConstants.KMinOutput, Constants.ClawConstants.KMaxOutput)
     .velocityFF(Constants.ClawConstants.clawKF)
     .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    motorConfig.closedLoop.maxMotion
      .maxVelocity(Constants.ClawConstants.maxVel)
      .maxAcceleration(Constants.ClawConstants.maxAcc)
      .allowedClosedLoopError(Constants.ClawConstants.allErr);
    
    //motorConfig.encoder.positionConversionFactor(360.0/gearRatio);

    clawElbow.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1;
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.21;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    clawLeft.getConfigurator().apply(slot0Configs);
    clawRight.getConfigurator().apply(slot0Configs);
  }

  





  //public 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elbow pos", motorEncoder.getPosition());
    SmartDashboard.putBoolean("Upper Sensor", SensorUpper.get());
    SmartDashboard.putBoolean("Lower Sensor", SensorLower.get());
    SmartDashboard.putNumber("Claw Left Vel", clawLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Claw Right Vel", clawRight.getVelocity().getValueAsDouble());
  }

  public void Claw_Grab(){
    clawLeft.setControl(m_request.withVelocity(40).withFeedForward(0));
    clawRight.setControl(m_request.withVelocity(-40).withFeedForward(0));
  }

  public void Claw_Release(){
    clawLeft.setControl(m_request.withVelocity(-30).withFeedForward(0));
    clawRight.setControl(m_request.withVelocity(30).withFeedForward(0));
  }

  public void Claw_ReleaseForAlae(){
    clawLeft.setControl(m_request.withVelocity(-65).withFeedForward(0));
    clawRight.setControl(m_request.withVelocity(65).withFeedForward(0));
  }

  public void Claw_ReleaseL1(){
    clawLeft.setControl(m_request.withVelocity(-50).withFeedForward(0));
    clawRight.setControl(m_request.withVelocity(50).withFeedForward(0));
  }

  public void Claw_FS(){
    clawLeft.setControl(m_request.withVelocity(-30).withFeedForward(0));
    clawRight.setControl(m_request.withVelocity(30).withFeedForward(0));
  }

  public void WristUp(){
    wristSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  
  public void WristDown(){
    wristSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void setPosition(double position){
    motorController.setReference(position, ControlType.kPosition);
  }

  public double getPosition(){
    return motorEncoder.getPosition();
  }

  public boolean getUpperSensor(){
    return SensorUpper.get();
  }

  public boolean getLowerSensor(){
    return SensorLower.get();
  }

  public void Claw_Stop(){
    clawLeft.setControl(m_request.withVelocity(0).withFeedForward(0));
    clawRight.setControl(m_request.withVelocity(0).withFeedForward(0));
  }

}
