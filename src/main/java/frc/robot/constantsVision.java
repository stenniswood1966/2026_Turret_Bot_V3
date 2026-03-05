// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class constantsVision {
    public static double k_PitchRollThreshold = 2;

    public class kg_Limelight1 {
        public static final String k_LimelightName = "limelight-front";
        public static final double k_LimelightFowardOffset = Units.inchesToMeters(-8.66);
        public static final double k_LimelightSideOffset = Units.inchesToMeters(8.24);
        public static final double k_LimelightUpOffset = Units.inchesToMeters(22.75);
        public static final double k_LimelightRollOffset = 0.0;
        public static final double k_LimelightPitchOffset = 20.0;
        public static final double k_LimelightYawOffset = 0.0;
    }
    public class kg_Limelight2 {
        public static final String k_LimelightName = "limelight-right";
        public static final double k_LimelightFowardOffset = Units.inchesToMeters(-12.18);
        public static final double k_LimelightSideOffset = Units.inchesToMeters(10.28);
        public static final double k_LimelightUpOffset = Units.inchesToMeters(22.75);
        public static final double k_LimelightRollOffset = 0.0;
        public static final double k_LimelightPitchOffset = 20;
        public static final double k_LimelightYawOffset = -120;
    }
    public class kg_Limelight3 {
        public static final String k_LimelightName = "limelight-left";
        public static final double k_LimelightFowardOffset = Units.inchesToMeters(-12.18);
        public static final double k_LimelightSideOffset = Units.inchesToMeters(6.22);
        public static final double k_LimelightUpOffset = Units.inchesToMeters(22.75);
        public static final double k_LimelightRollOffset = 0.0;
        public static final double k_LimelightPitchOffset = 20.0;
        public static final double k_LimelightYawOffset = 120;
    }
}