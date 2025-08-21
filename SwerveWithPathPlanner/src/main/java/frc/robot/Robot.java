// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final boolean kUseLimelight = true;

  //We create the Field2d object to visualize the robot's position on the field in the dashboard.
  private final Field2d m_field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putData("Field", m_field);
  }
     
  @Override
  public void robotPeriodic() { //Robot periodic is called every 20ms, so we can use it to update the dashboard with the robot's position and other information.
    CommandScheduler.getInstance().run();

    //We get the robot's pose from the drivetrain and set it in the Field2d object to visualize it on the dashboard.
    m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose); 
    
    if (kUseLimelight) {
      //We get the robot's heading and angular velocity from the drivetrain state.
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      //We send the robot's orientation to the Limelight camera to help it calculate the position of the tags with MegaTag2.
      //We get the robot's pose estimate back from the Limelight camera.
      LimelightHelpers.SetRobotOrientation("limelight-right", headingDeg, 0, 0, 0, 0, 0);
      var llrMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
      if (llrMeasurement != null && llrMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        // If the Limelight camera has a valid pose estimate, we add it to the drivetrain's vision measurements. PENDING: Add standar deviation to the vision measurement for better filtering.
        m_robotContainer.drivetrain.addVisionMeasurement(llrMeasurement.pose, llrMeasurement.timestampSeconds);
      }
      //We do the same for the left Limelight camera.
      LimelightHelpers.SetRobotOrientation("limelight-left", headingDeg, 0, 0, 0, 0, 0);
      var lllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
      if (lllMeasurement != null && lllMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(lllMeasurement.pose, lllMeasurement.timestampSeconds);
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
