// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final class ClawConstants {
            public static final int clawElbowID = 25;
            public static final int clawLeftID = 8;
            public static final int clawRightID = 9;
            public static final int sensorUpperID = 2;
            public static final int sensorLowerID = 1;
            public static final int PH_CAN_ID = 40;
            public static final int solenoidForwardChannel = 7;
            public static final int solenoidReverseChannel = 6;

            public static final double clawKP = 0.15;
            public static final double clawKI = 0.0;
            public static final double clawKD = 0.0;
            public static final double clawKF = 0.0;
            public static final double KMinOutput = -1.0;
            public static final double KMaxOutput = 1.0;
            public static final double maxVel = 1;
            public static final double maxAcc = 1;
            public static final double allErr = 1;



            public static final double default_pos = 3.3;
            public static final double feedCoral_pos = 0.5;
            public static final double groundAlgae_pos = 0;
            public static final double L1_pos = 0;
            public static final double L2_pos = 7.5; 
            public static final double L3_pos = 0;
            public static final double L4_pos = 0;
            public static final double P_pos = 0;
            public static final double N_pos = 0;
            public static final double A1_pos = 0;
            public static final double A2_pos = 0;

            public static final double slot0S = 0.1;
            public static final double slot0V = 0.12;
            public static final double slot0A = 0.0012;
            public static final double slot0P = 0.21;
            public static final double slot0I = 0.0;
            public static final double slot0D = 0.0;

    }

    public static final class ElevatorConstants {
        public static final int elevatorLeftID = 17;
        public static final int elevatorRightID = 10;
        public static final double elevatorKP = 0.1;
        public static final double elevatorKI = 0.0000075;
        public static final double elevatorKD = 0.0;
        public static final double elevatorKFF = 0.0;
        public static final double KMinOutput = -1.0;
        public static final double KMaxOutput = 1.0;
        public static final double maxVel = 1;
        public static final double maxAcc = 1;
        public static final double allErr = 1;

        //LEFT MOTOR POSITIONS
        public static final double leftDefault_pos = 0;
        public static final double leftFS_pos = 0;
        public static final double leftFloorAlgae_pos = 0;

        public static final double leftL1_pos = 0;
        public static final double leftL2_pos = 19.38;
        public static final double leftL3_pos = 0;
        public static final double leftL4_pos = 0;

        public static final double leftProcessor_pos = 0;
        public static final double leftNet_pos = 0;
        public static final double leftA1_pos = 0;
        public static final double leftA2_pos = 0;

        //RIGHT MOTOR POSITIONS
        public static final double rightDefault_pos = 0;
        public static final double rightFS_pos = 0;
        public static final double rightFloorAlgae_pos = 0;

        public static final double rightL1_pos = 0;
        public static final double rightL2_pos = -19.38;
        public static final double rightL3_pos = 0;
        public static final double rightL4_pos = 0;

        public static final double rightProcessor_pos = 0;
        public static final double rightNet_pos = 0;
        public static final double rightA1_pos = 0;
        public static final double rightA2_pos = 0;
        
    }

}
