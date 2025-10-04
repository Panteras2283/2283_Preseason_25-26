package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.VecBuilder;

import java.io.IOException;
import java.util.HashMap;  
import java.util.List;
import java.util.Map;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private String selectedPoseKey = "1";
    private final Map<String, List<Pose2d>> poseMap;
    private final Map<String, List<PathPlannerPath>> pathMap;


    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

        this.setVisionMeasurementStdDevs(
            VecBuilder.fill(
                0.30,
                0.30,
                0.35
            )
        );

        poseMap = buildPoseMap();
        pathMap = buildPathMap();

        SmartDashboard.putString("Selected Pose", selectedPoseKey);
    }

    private Map<String, List<Pose2d>> buildPoseMap(){
        Map<String, List<Pose2d>> map = new HashMap<>();
        Rotation2d one = Rotation2d.fromDegrees(180);
        Rotation2d two = Rotation2d.fromDegrees(-120);
        Rotation2d three = Rotation2d.fromDegrees(-60);
        Rotation2d four = Rotation2d.fromDegrees(0);
        Rotation2d five = Rotation2d.fromDegrees(60);
        Rotation2d six = Rotation2d.fromDegrees(120);

        map.put("1", List.of(
            new Pose2d(14.386521979123565,3.8652093619149492,one),
            new Pose2d(14.37308854121592, 4.022884768621724,one),
            new Pose2d(14.369115437656363, 4.1617740410172575,one)
        ));
        map.put("2", List.of(
            new Pose2d(13.927357713135624, 5.077049976831412,two),
            new Pose2d(13.733098481974052, 5.177925337234092,two),
            new Pose2d(13.596906024202786, 5.2508740999154675,two)
        ));
        map.put("3", List.of(
            new Pose2d(0,0,three),
            new Pose2d(0,0,three),
            new Pose2d(0,0,three)
        ));
        map.put("4", List.of(
            new Pose2d(0,0,four),
            new Pose2d(0,0,four),
            new Pose2d(0,0,four)
        ));
        map.put("5", List.of(
            new Pose2d(0,0,five),
            new Pose2d(0,0,five),
            new Pose2d(0,0,five)
        ));
        map.put("6", List.of(
            new Pose2d(13.557424187425514, 2.773494019391029,six),
            new Pose2d(13.687912422277794, 2.8704204132661006,six),
            new Pose2d(13.864246378035794, 2.949463964566611,six)
        ));

        return map;
    }

    private Map<String, List<PathPlannerPath>> buildPathMap(){
        Map<String, List<PathPlannerPath>> map = new HashMap<>();
        //PathPlannerPath one_L_R = PathPlannerPath.fromPathFile("1_L_R");
      
        try {
            map.put("1", List.of(
                PathPlannerPath.fromPathFile("1_L"),
                PathPlannerPath.fromPathFile("1_C"),
                PathPlannerPath.fromPathFile("1_R")
            ));
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        try {
            map.put("2", List.of(
                PathPlannerPath.fromPathFile("2_L"),
                PathPlannerPath.fromPathFile("2_C"),
                PathPlannerPath.fromPathFile("2_R")
            ));
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        try {
            map.put("3", List.of(
                PathPlannerPath.fromPathFile("1_L"),
                PathPlannerPath.fromPathFile("1_C"),
                PathPlannerPath.fromPathFile("1_R")
            ));
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        try {
            map.put("4", List.of(
                PathPlannerPath.fromPathFile("1_L"),
                PathPlannerPath.fromPathFile("1_C"),
                PathPlannerPath.fromPathFile("1_R")
            ));
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        try {
            map.put("5", List.of(
                PathPlannerPath.fromPathFile("1_L"),
                PathPlannerPath.fromPathFile("1_C"),
                PathPlannerPath.fromPathFile("1_R")
            ));
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        try {
            map.put("6", List.of(
                PathPlannerPath.fromPathFile("6_L"),
                PathPlannerPath.fromPathFile("6_C"),
                PathPlannerPath.fromPathFile("6_R")
            ));
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        return map;
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(3.6, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(2, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public Command PathfindToPose(
        Pose2d targetPose,
        PathConstraints constraints,
        double goalEndVelocity
    ) {
       Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            goalEndVelocity
        );
        pathfindingCommand.addRequirements(this);
        return pathfindingCommand;
    }

    public Command PathfindToReef(
        int alignmentIndex,
        PathConstraints constraints,
        double goalEndVelocity
    ) {
       Command pathfindingCommand = AutoBuilder.pathfindToPose(
            poseMap.get(selectedPoseKey).get(alignmentIndex),
            constraints,
            goalEndVelocity
        );
        pathfindingCommand.addRequirements(this);
        return pathfindingCommand;
    }

    public Command PathfindToReefpath(
        int alignmentIndex,
        PathConstraints constraints,
        double goalEndVelocity
    ) {
       Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            pathMap.get(selectedPoseKey).get(alignmentIndex),
            constraints
        );
        pathfindingCommand.addRequirements(this);
        return pathfindingCommand;
    }

    private void setPoseKey(String key){
        selectedPoseKey = key;
        SmartDashboard.putString("Selected Pose", selectedPoseKey);
    }

    private void autoSelectPoseKey(){
            double heading = getState().Pose.getRotation().getDegrees();
            double face1 = 180;
            double face2 = -120;
            double face3 = -60;
            double face4 = 0;
            double face5 = 60;
            double face6 = 120;
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red){
            if (heading <= face1 +30 && heading >= face1 -30){
                setPoseKey("1");
            }
            else if (heading <= face2 +30 && heading >= face2 -30){
                setPoseKey("2");
            }
            else if (heading <= face3 +30 && heading >= face3 -30){
                setPoseKey("3");
            }
            else if (heading <= face4 +30 && heading >= face4 -30){
                setPoseKey("4");
            }
            else if (heading <= face5 +30 && heading >= face5 -30){
                setPoseKey("5");
            }
            else if (heading <= face6 +30 && heading >= face6 -30){
                setPoseKey("6");
            }
        }
        else{
            if (heading <= face1 +30 && heading >= face1 -30){
                setPoseKey("4");
            }
            else if (heading <= face2 +30 && heading >= face2 -30){
                setPoseKey("5");
            }
            else if (heading <= face3 +30 && heading >= face3 -30){
                setPoseKey("6");
            }
            else if (heading <= face4 +30 && heading >= face4 -30){
                setPoseKey("1");
            }
            else if (heading <= face5 +30 && heading >= face5 -30){
                setPoseKey("2");
            }
            else if (heading <= face6 +30 && heading >= face6 -30){
                setPoseKey("3");
            }

        }
    }



    @Override
    public void periodic() {

        autoSelectPoseKey();
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    

    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

}
