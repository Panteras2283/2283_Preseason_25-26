// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
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
import edu.wpi.first.wpilibj.RobotBase;
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
  public final RelativeEncoder motorEncoder = clawElbow.getEncoder();

  private final DoubleSolenoid wristSolenoid =
  new DoubleSolenoid(Constants.ClawConstants.PH_CAN_ID, 
  PneumaticsModuleType.CTREPCM, 
  Constants.ClawConstants.solenoidForwardChannel, 
  Constants.ClawConstants.solenoidReverseChannel);

  // Simulation-specific objects
  private SparkFlexSim clawElbowSim;
  private double simTargetPosition = 0;
  private double m_simCurrentPosition = 0; // The actual simulated position
  private static final double SIM_SPEED = 50.0; // Units per second, adjust as needed
  private TalonFXSimState leftGripperSim = clawLeft.getSimState();
  private TalonFXSimState rightGripperSim = clawRight.getSimState();
  private double leftGripperTargetVelocity = 0;
  private double rightGripperTargetVelocity = 0;

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
    
    clawElbow.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = Constants.ClawConstants.slot0P;
    slot0Configs.kI = Constants.ClawConstants.slot0I;
    slot0Configs.kD = Constants.ClawConstants.slot0D;
    slot0Configs.kS = Constants.ClawConstants.slot0S;
    slot0Configs.kV = Constants.ClawConstants.slot0V;
    slot0Configs.kA = Constants.ClawConstants.slot0A;

    clawLeft.getConfigurator().apply(slot0Configs);
    clawRight.getConfigurator().apply(slot0Configs);

    if (RobotBase.isSimulation()) {
            clawElbowSim = new SparkFlexSim(clawElbow, DCMotor.getNeoVortex(1));

        } else {
            clawElbowSim = null;
        }
  }

  @Override
  public void periodic() {

    if (RobotBase.isReal()){
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("Elbow pos", motorEncoder.getPosition());
      SmartDashboard.putBoolean("Upper Sensor", SensorUpper.get());
      SmartDashboard.putBoolean("Lower Sensor", SensorLower.get());
      SmartDashboard.putNumber("Claw Left Vel", clawLeft.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Claw Right Vel", clawRight.getVelocity().getValueAsDouble());
    }
    if (RobotBase.isSimulation()){
       // Move the simulated position towards the target
      double delta = SIM_SPEED * 0.02; // 0.02 is the periodic loop time (20ms)
      if (Math.abs(simTargetPosition - m_simCurrentPosition) > delta) {
        // If we are not at the target, move towards it
        m_simCurrentPosition += Math.signum(simTargetPosition - m_simCurrentPosition) * delta;
      } else {
        // If we are close, snap to the target to avoid small oscillations
        m_simCurrentPosition = simTargetPosition;
        }
      clawElbowSim.setPosition(m_simCurrentPosition);
      leftGripperSim.setRotorVelocity(leftGripperTargetVelocity);
      rightGripperSim.setRotorVelocity(rightGripperTargetVelocity);
      
      SmartDashboard.putNumber("Elbow pos", clawElbowSim.getPosition());
      SmartDashboard.putNumber("Claw Left Vel", clawLeft.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Claw Right Vel", clawRight.getVelocity().getValueAsDouble());
    }

  }

  public void Claw_Grab(){
    clawLeft.setControl(m_request.withVelocity(40));
    clawRight.setControl(m_request.withVelocity(-40));

    if (RobotBase.isSimulation()) {
      leftGripperTargetVelocity = 40;
      rightGripperTargetVelocity = -40;
    }
  }

  public void Claw_Release(){
    clawLeft.setControl(m_request.withVelocity(-30));
    clawRight.setControl(m_request.withVelocity(30));

    if (RobotBase.isSimulation()) {
      leftGripperTargetVelocity = -30;
      rightGripperTargetVelocity = 30;
    }
  }

  public void Claw_ReleaseForAlae(){
    clawLeft.setControl(m_request.withVelocity(-65));
    clawRight.setControl(m_request.withVelocity(65));

    if (RobotBase.isSimulation()) {
      leftGripperTargetVelocity = -65;
      rightGripperTargetVelocity = 65;
    }
  }

  public void Claw_ReleaseL1(){
    clawLeft.setControl(m_request.withVelocity(-50));
    clawRight.setControl(m_request.withVelocity(50));

    if (RobotBase.isSimulation()) {
      leftGripperTargetVelocity = -50;
      rightGripperTargetVelocity = 50;
    }
  }

  public void Claw_FS(){
    clawLeft.setControl(m_request.withVelocity(-30));
    clawRight.setControl(m_request.withVelocity(30));

    if (RobotBase.isSimulation()) {
      leftGripperTargetVelocity = -30;
      rightGripperTargetVelocity = 30;
    }
  }

  public void WristUp(){
    wristSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  
  public void WristDown(){
    wristSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void setPosition(double position){
    if (RobotBase.isSimulation()) {
      simTargetPosition = position;
    }
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
    clawLeft.setControl(m_request.withVelocity(0));
    clawRight.setControl(m_request.withVelocity(0));

    if (RobotBase.isSimulation()) {
      leftGripperTargetVelocity = 0;
      rightGripperTargetVelocity = 0;
    }
  }

}
