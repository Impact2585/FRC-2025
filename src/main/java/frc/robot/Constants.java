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
        public static final int elevator1CanID = 41;
        public static final int elevator2CanID = 42;
        public static final int rollerCanID = 31;
    
        public static final double rollerSpeed = 0.5;
        public static final double elevatorSpeed = 0.05;
    
        public static final double elevatorLowStop = 0;
        public static final double elevatorHighStop = 120;
    
        public static final double eP = 0.04;
        public static final double eI = 0.002;
        public static final double eD = 0;
    
        //public static final double loweredPos = 0;
        public static final double l1 = 0;
        public static final double l2 = 6.5;
        public static final double l3 = 17.25;
        public static final double l4 = 32;
        
      }

      public static final class AlgaeRollersConstants{
        public static final int lowerAlgaeRollersMotorCanId = 11;
        public static final int upperAlgaeRollersMotorCanId = 12;
    
        public static final double maxSpeed = 0.4;
      }

      public static final class CoralRollersConstants{
        public static final int chuteMotor1CanId = 13;
        public static final int chuteMotor2CanId = 14;

        public static final int coralRoller1CanId = 21;
        public static final int coralRoller2CanId = 22;
    
        public static final double chuteSpeed = .7;
        public static final double rollerSlowSpeed = 0.1;
        public static final double rollerFastSpeed = 0.4;
      }
}