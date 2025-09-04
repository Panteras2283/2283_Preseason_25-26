// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
            public static final int clawLeftID = 8;
            public static final int clawRightID = 9;
            public static final int sensorUpperID = 2;
            public static final int sensorLowerID = 1;
            public static final int PH_CAN_ID = 6;
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
           // public static final double  = 0.05;
    }

}
