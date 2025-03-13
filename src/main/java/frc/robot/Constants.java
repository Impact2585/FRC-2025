// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

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
    
        public static final double rollerSpeed = 0.5;
        public static final double elevatorSpeed = 0.2;
    
        public static final double elevatorLowStop = 0;
        public static final double elevatorHighStop = 120;
    
        public static final double eP = 0.04;
        public static final double eI = 0.005;
        public static final double eD = 0;

        //public static final double loweredPos = 0;
        public static final double l1 = -0.2;
        public static final double l2 = 5.2;
        public static final double l3 = 15.15;
        public static final double l4 = 32;
        public static final double l3_5 = 25.0;
        public static final double l2_5 = 15.5;
      }

      public static final class AlgaeRollersConstants{
        public static final int lowerAlgaeRollersMotorCanId = 31;
        public static final int upperAlgaeRollersMotorCanId = 32;
    
        public static final double maxSpeed = 0.75;
      }

      public static final class CoralRollersConstants{
        public static final int chuteMotor1CanId = 51;
        public static final int chuteMotor2CanId = 52;

        public static final int coralRoller1CanId = 21;
        public static final int coralRoller2CanId = 22;
    
        public static final double chuteSpeed = .7;
        public static final double rollerSlowSpeed = 0.1;
        public static final double rollerFastSpeed = 0.3;
      }

      public static final class ClimbConstants{
        public static final int climbMotorCanId = 13;
        public static final double climbSpeed = 1.0;
        public static final double climbMax = 15.0;
      }
      public static final class VisionConstants{
        public static final Pose3d robotToCamera = new Pose3d(0.0,0.0,0.0, new Rotation3d(0.0,0.0,0.0));
        public static final double distanceToTag = 1;
      }

      public static final double X_REEF_ALIGNMENT_P = 3.3;
      public static final double Y_REEF_ALIGNMENT_P = 3.3;
      public static final double ROT_REEF_ALIGNMENT_P = 0.058;
    
      public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
      public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
      public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;  // Vertical pose
      public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
      public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
      public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;
    
      public static final double DONT_SEE_TAG_WAIT_TIME = 1;
      public static final double POSE_VALIDATION_TIME = 0.3;
}