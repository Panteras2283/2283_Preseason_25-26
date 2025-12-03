// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.util.*;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final boolean kUseLimelightLeft = true;
  private final boolean kUseLimelightRight = true;
  
  //Std devs constants
  private static final double kSpinGateRevPerSec = 1.0;

  private static final double kDisagreePosM = 0.12;
  private static final double kDisagreePosMaxM = 0.5;
  private static final double kDisagreeThRad = Math.toRadians(7.5);
  private static final double kDisagreeThMaxRad = Math.toRadians(23);

  private static final double kBaseStdX = 0.18;
  private static final double kBaseStdY = 0.18;
  private static final double kBaseStdTh = 0.22;

  private static final int kQualityReqTags = 2;

  private static final double kFlowMaxMps = 4.0;
  private static final double kFlowMaxScale = 1.8;

  private static final double kStdXYFloor = 0.06;
  private static final double kStdXYCeil = 1.20;
  private static final double kStdThFloor = 0.10;
  private static final double kStdThCeil = 1.20;

  private static final double kTrustMaxFactor = 0.65;
  private static final double kTrustMinFactor = 0.30;
  //Std devs constants end
  //Std devs helpers
  private Matrix<N3, N1> visionStdDevsAdaptive(LimelightHelpers.PoseEstimate est, Pose2d curPose, double vTransMps){
    int tags = est.tagCount;
    boolean qualityGood = (tags >= kQualityReqTags);

    double dx = est.pose.getX() - curPose.getX();
    double dy = est.pose.getY() - curPose.getY();
    double posErr = Math.hypot(dx, dy);
    double dtheta = Math.abs(est.pose.getRotation().minus(curPose.getRotation()).getRadians());

    boolean bigDisagreement = (posErr > kDisagreePosM) || (dtheta > kDisagreeThRad);

    double baseX = kBaseStdX, baseY = kBaseStdY, baseTh = kBaseStdTh;
    
    double qualityScale = 1.0;
    if (tags >= kQualityReqTags + 1) qualityScale *= 0.85;
    else if (tags == 1) qualityScale *= 1.10;

    double trustBoost = 1.0;
    if(qualityGood && bigDisagreement){
      double posFactor = mapClamp(posErr, kDisagreePosM, kDisagreePosMaxM, kTrustMaxFactor, kTrustMinFactor);
      double thFactor = mapClamp(dtheta, kDisagreeThRad, kDisagreeThMaxRad, kTrustMaxFactor, kTrustMinFactor);
      trustBoost = Math.min(posFactor, thFactor);
    }

    double flowScale = mapClamp(vTransMps, 0.0, kFlowMaxMps, 1.0, kFlowMaxScale);

    double sx = clamp(baseX * qualityScale * trustBoost * flowScale, kStdXYFloor, kStdXYCeil);
    double sy = clamp(baseY * qualityScale * trustBoost * flowScale, kStdXYFloor, kStdXYCeil);
    double sth = clamp(baseTh * qualityScale * trustBoost, kStdThFloor, kStdThCeil);

    return VecBuilder.fill(sx, sy, sth);
  }

  private static double clamp(double v, double lo, double hi){
    return Math.max(lo, Math.min(hi, v));
  }
  private static double mapClamp(double x, double inLo, double inHi, double outLo, double outHi){
    double t = (x - inLo) / (inHi - inLo);
    t = clamp(t, 0.0, 1.0);
    return outLo + (outHi - outLo) * t;
  }


  //private final QuestNav questNav = new QuestNav();
  
  
  
  //We create the Field2d object to visualize the robot's position on the field in the dashboard.
  private final Field2d m_field = new Field2d();

  

  public Robot() {
    m_robotContainer = new RobotContainer();

    // Add Field2d to SmartDashboard
    SmartDashboard.putData("Field", m_field);

    // Add this line to put the scheduler on SmartDashboard
    SmartDashboard.putData(CommandScheduler.getInstance());
    

    // Add boolean Limelight buttons to SmartDashboard
    SmartDashboard.putBoolean("LL Left", kUseLimelightLeft);
    SmartDashboard.putBoolean("LL Right", kUseLimelightRight);

    var driveState = m_robotContainer.drivetrain.getState();

    // Add Swerve Drive states to SmartDashboard
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("SwerveDrive");

      builder.addDoubleProperty("Front Left Angle", () -> driveState.ModuleStates[0].angle.getRadians(), null);
      builder.addDoubleProperty("Front Left Velocity", () -> driveState.ModuleStates[0].speedMetersPerSecond, null);

      builder.addDoubleProperty("Front Right Angle", () -> driveState.ModuleStates[1].angle.getRadians(), null);
      builder.addDoubleProperty("Front Right Velocity", () -> driveState.ModuleStates[1].speedMetersPerSecond, null);

      builder.addDoubleProperty("Back Left Angle", () -> driveState.ModuleStates[2].angle.getRadians(), null);
      builder.addDoubleProperty("Back Left Velocity", () -> driveState.ModuleStates[2].speedMetersPerSecond, null);

      builder.addDoubleProperty("Back Right Angle", () -> driveState.ModuleStates[3].angle.getRadians(), null);
      builder.addDoubleProperty("Back Right Velocity", () -> driveState.ModuleStates[3].speedMetersPerSecond, null);

      builder.addDoubleProperty("Robot Angle", () -> (Math.PI) + driveState.Pose.getRotation().getRadians(), null);
  }
  });
  
  }
     
  @Override
  public void robotPeriodic() { //Robot periodic is called every 20ms, so we can use it to update the dashboard with the robot's position and other information.
    CommandScheduler.getInstance().run();
    
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());


    //We get the robot's pose from the drivetrain and set it in the Field2d object to visualize it on the dashboard.
    m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose); 

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
    // Display Pathplanner pose
    m_field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
    // Disply path in field2d object
    m_field.getObject("path").setPoses(poses);
    });


    //We get the robot's heading and angular velocity from the drivetrain state.
    var driveState = m_robotContainer.drivetrain.getState();
    double headingDeg = driveState.Pose.getRotation().getDegrees();
    double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    //Limelight Pose Estimations
    if (kUseLimelightRight) {
      //We send the robot's orientation to the Limelight camera to help it calculate the position of the tags with MegaTag2.
      //We get the robot's pose estimate back from the Limelight camera.
      LimelightHelpers.SetRobotOrientation("limelight-right", headingDeg, 0, 0, 0, 0, 0);
      var llrMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
      if (llrMeasurement != null && llrMeasurement.tagCount > 0) {
        var stNow = m_robotContainer.drivetrain.getState();
        double vTransMps = Math.hypot(stNow.Speeds.vxMetersPerSecond, stNow.Speeds.vyMetersPerSecond);

        if (Math.abs(omegaRps) < kSpinGateRevPerSec) {
          var std = visionStdDevsAdaptive(llrMeasurement, stNow.Pose, vTransMps);
          m_robotContainer.drivetrain.addVisionMeasurement(llrMeasurement.pose, llrMeasurement.timestampSeconds, std);
        } 
      
      }
    }

    if (kUseLimelightLeft) {
      //We do the same for the left Limelight camera.
      LimelightHelpers.SetRobotOrientation("limelight-left", headingDeg, 0, 0, 0, 0, 0);
      var lllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
      if (lllMeasurement != null && lllMeasurement.tagCount > 0) {
        var stNow = m_robotContainer.drivetrain.getState();
        double vTransMps = Math.hypot(stNow.Speeds.vxMetersPerSecond, stNow.Speeds.vyMetersPerSecond);
        
        if (Math.abs(omegaRps) < kSpinGateRevPerSec) {
          var std = visionStdDevsAdaptive(lllMeasurement, stNow.Pose, vTransMps);
          m_robotContainer.drivetrain.addVisionMeasurement(lllMeasurement.pose, lllMeasurement.timestampSeconds, std);
        }
       
      }
    }

    

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    // Add this line to switch to the "Autonomous" tab on Elastic
    Elastic.selectTab("Autonomous");
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // Add this line to switch to the "Teleoperated" tab on Elastic
    Elastic.selectTab("Teleoperated");
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


  @Override
  public void simulationPeriodic() {}
}
