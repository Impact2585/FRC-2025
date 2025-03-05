package frc.robot.commands;


import static frc.robot.Constants.VisionConstants.robotToCamera;
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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class ChaseTagCommand extends Command {
 
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedLimiter = 0.5;
    private static final Transform3d TAG_TO_GOAL =
        new Transform3d(
            new Translation3d(1.5, 0.0, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI));


    private final CommandSwerveDrivetrain drivetrainSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("");
    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);


    public ChaseTagCommand(CommandSwerveDrivetrain drivetrainSubsystem, Supplier<Pose2d> poseProvider) {


        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseProvider = poseProvider;


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
                    robotToCamera.getX(),robotToCamera.getY(),0,
                    0,0,0);
        Pose2d robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }


    @Override
    public void execute() {
        //Check if 
        Pose2d robotPose2d = poseProvider.get();
        Pose3d robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(),0.0, 
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        if (LimelightHelpers.getTV("")) {
            // This is new target data, so recalculate the goal
            double[] targetPoseArray = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
            Pose3d targetPose = new Pose3d(targetPoseArray[0], targetPoseArray[1], targetPoseArray[2],
                                        new Rotation3d(targetPoseArray[3], targetPoseArray[4], targetPoseArray[6]));
            // Transform the tag's pose to set our goal
            var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();


            // Set goal on PID
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
        }
    
        if (!LimelightHelpers.getTV("")) {
        // No target has been visible
            // drivetrainSubsystem.stop();
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
            System.out.println("xSpeed: "+ xSpeed+"\nySpeed: "+ySpeed +"\nrotation: " + omegaSpeed);
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

