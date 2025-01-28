// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ElevatorConstants{
        public static final int elevatorCanID = 41;
        public static final int rollerCanID = 42;
    
        public static final double rollerSpeed = 0.5;
        public static final double elevatorSpeed = 0.4;
    
        public static final double elevatorLowStop = 0;
        public static final double elevatorHighStop = 120;
    
        public static final double eP = 0.05;
        public static final double eI = 0;
        public static final double eD = 0;
    
        public static final double loweredPos = 0;
        public static final double sourcePos = 50;
      }
}