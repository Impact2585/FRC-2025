package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.AlgaeRollersConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AlgaeRollers extends SubsystemBase {

    private SparkMaxConfig algaerollersconfig = new SparkMaxConfig();
    private SparkMax AlgaeRoller1 = new SparkMax(AlgaeRollersConstants.lowerAlgaeRollersMotorCanId, MotorType.kBrushless);
    private SparkMax AlgaeRoller2 = new SparkMax(AlgaeRollersConstants.upperAlgaeRollersMotorCanId, MotorType.kBrushless);

    private double AlgaeRollersStatus;

    public AlgaeRollers() {
        algaerollersconfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
        stopAlgaeRollers();

        AlgaeRoller1.configure(algaerollersconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        AlgaeRoller2.configure(algaerollersconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    public void spinIn() {
        //System.out.println("spinning");
        AlgaeRoller1.set(AlgaeRollersConstants.maxSpeed);
        AlgaeRoller2.set(-AlgaeRollersConstants.maxSpeed);
    }

    public void spinOut(){
        //System.out.println("spinning out");
        AlgaeRoller1.set(-AlgaeRollersConstants.maxSpeed);
        AlgaeRoller2.set(AlgaeRollersConstants.maxSpeed);
    }

    public void stopAlgaeRollers(){
        AlgaeRoller1.set(0);
        AlgaeRoller2.set(0);
    }
}