package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.AlgaeRollersConstants;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AlgaeRollers extends SubsystemBase {

    private SparkMaxConfig algaerollersconfig = new SparkMaxConfig();
    private SparkMax AlgaeRollersLowerMotor = new SparkMax(AlgaeRollersConstants.lowerAlgaeRollersMotorCanId, MotorType.kBrushless);
    private SparkMax AlgaeRollersUpperMotor = new SparkMax(AlgaeRollersConstants.upperAlgaeRollersMotorCanId, MotorType.kBrushless);

    private double AlgaeRollersStatus;

    public AlgaeRollers() {
        algaerollersconfig
            .smartCurrentLimit(20);
        stopAlgaeRollers();

        AlgaeRollersLowerMotor.configure(algaerollersconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        AlgaeRollersUpperMotor.configure(algaerollersconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    public void spinIn() {
        System.out.println("spinning");
        AlgaeRollersLowerMotor.set(AlgaeRollersConstants.maxSpeed);
        AlgaeRollersUpperMotor.set(AlgaeRollersConstants.maxSpeed);
    }

    public void spinOut(){
        System.out.println("spinning out");
        AlgaeRollersLowerMotor.set(-AlgaeRollersConstants.maxSpeed);
        AlgaeRollersUpperMotor.set(-AlgaeRollersConstants.maxSpeed);
    }

    public void stopAlgaeRollers(){
        AlgaeRollersLowerMotor.set(0);
        AlgaeRollersUpperMotor.set(0);
    }
}