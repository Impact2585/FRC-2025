package frc.robot.commands;


import static frc.robot.Constants.VisionConstants;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class AlignToTagRotation extends Command {
 
    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private static double speedLimiter = 0.2;
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(MaxSpeed, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(MaxSpeed, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularRate-2);
    
    private static final Transform3d TAG_TO_GOAL =
        new Transform3d(
            new Translation3d(VisionConstants.distanceToTag, 0.0, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI));


    private final CommandSwerveDrivetrain drivetrainSubsystem;
    // private final Supplier<Pose2d> poseProvider;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("");
    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);


    public AlignToTagRotation(CommandSwerveDrivetrain drivetrainSubsystem) {


        this.drivetrainSubsystem = drivetrainSubsystem;
        // this.poseProvider = poseProvider;


        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);


        addRequirements(drivetrainSubsystem);
    }


    @Override
    public void initialize() {
        // lastTarget = null;
        LimelightHelpers.setCameraPose_RobotSpace(getName(),
                    VisionConstants.robotToCamera.getX(),VisionConstants.robotToCamera.getY(),0,
                    0,0,0);
        // Pose2d robotPose = poseProvider.get();
        omegaController.reset(0);
        xController.reset(0);
        yController.reset(0);
    }


    @Override
    public void execute() {
        //Check if 
        Pose2d robotPose2d = new Pose2d(0.0,0.0,new Rotation2d(0.0,0.0));
        Pose3d robotPose = new Pose3d(0.0,0.0,0.0, 
                new Rotation3d(0.0, 0.0, 0.0));

        if (LimelightHelpers.getTV("")) {
            // This is new target data, so recalculate the goal
            double[] targetPoseArray = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
            Pose3d targetPose = new Pose3d(targetPoseArray[0], targetPoseArray[1], targetPoseArray[2],
                                        new Rotation3d(targetPoseArray[3], targetPoseArray[4], targetPoseArray[5]));
            // Transform the tag's pose to set our goal
            var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();


            // Set goal on PID
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
        }
    
        if (!LimelightHelpers.getTV("")) {
        // No target has been visible
            // drivetrainSubsystem.applyRequest(() ->
            //     drive.withVelocityX(0.0) // Drive forward with negative Y (forward)
            //         .withVelocityY(0.0) // Drive left with negative X (left)
            //         .withRotationalRate(0.0) // Drive counterclockwise with negative X (left)
            // )
            System.out.println("no target, not moving");
        } else {
        // Calculate speeds
            double xSpeed = xController.calculate(robotPose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }

            double ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }
            
            double omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }
        //Drive to target
            // drivetrainSubsystem.applyRequest(() ->
            //     drive.withVelocityX(xSpeed * speedLimiter) // Drive forward with negative Y (forward)
            //         .withVelocityY(ySpeed * speedLimiter) // Drive left with negative X (left)
            //         .withRotationalRate(omegaSpeed * speedLimiter) // Drive counterclockwise with negative X (left)
            // )
            SmartDashboard.putNumber("Pose X", robotPose.getX());
            SmartDashboard.putNumber("Pose Y", robotPose.getY());
            SmartDashboard.putNumber("Pose Angle", robotPose2d.getRotation().getRadians());
        }


    }


    @Override
    public void end(boolean interrupted) {
        // drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        if(xController.atGoal() && yController.atGoal() && omegaController.atGoal()){
            return true;
        } else {
            return false;
        }
    }

}

