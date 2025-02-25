package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.CoralRollersConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CoralRollers extends SubsystemBase {
    //1 = left (facing forwards), 2 = right (facing forwards)
    private SparkMaxConfig coralRollerMotorConfig = new SparkMaxConfig();
    //private SparkMax ChuteMotor1 = new SparkMax(CoralRollersConstants.chuteMotor1CanId, MotorType.kBrushless);
    //private SparkMax ChuteMotor2 = new SparkMax(CoralRollersConstants.chuteMotor2CanId, MotorType.kBrushless);
    private SparkMax CoralRoller1 = new SparkMax(CoralRollersConstants.coralRoller1CanId, MotorType.kBrushless);
    private SparkMax CoralRoller2 = new SparkMax(CoralRollersConstants.coralRoller2CanId, MotorType.kBrushless);

    private double CoralRollersStatus;

    public CoralRollers() {
        coralRollerMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);

        //ChuteMotor1.configure(coralRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //ChuteMotor2.configure(coralRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        CoralRoller1.configure(coralRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        CoralRoller2.configure(coralRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    public void preRoller() {
        System.out.println("rolling");
        //ChuteMotor1.set(-CoralRollersConstants.chuteSpeed);
        //ChuteMotor2.set(CoralRollersConstants.chuteSpeed);
        CoralRoller1.set(CoralRollersConstants.rollerSlowSpeed);
        CoralRoller2.set(-CoralRollersConstants.rollerSlowSpeed);
    }

    public void scoreOut(){
        //System.out.println("rolling");
        CoralRoller1.set(CoralRollersConstants.rollerFastSpeed);
        CoralRoller2.set(-CoralRollersConstants.rollerFastSpeed);
    }

    public void stopCoralRollers(){
        CoralRoller1.set(0);
        CoralRoller2.set(0);
        //ChuteMotor1.set(0);
        //ChuteMotor2.set(0);
    }
}