package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.CoralRollersConstants;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CoralRollers extends SubsystemBase {

    private SparkMaxConfig Coralrollersconfig = new SparkMaxConfig();
    private SparkMax CoralRollersLowerMotor = new SparkMax(CoralRollersConstants.lowerCoralRollersMotorCanId, MotorType.kBrushless);
    private SparkMax CoralRollersUpperMotor = new SparkMax(CoralRollersConstants.upperCoralRollersMotorCanId, MotorType.kBrushless);

    private double CoralRollersStatus;

    public CoralRollers() {
        Coralrollersconfig
            .smartCurrentLimit(20);
        stopCoralRollers();

        CoralRollersLowerMotor.configure(Coralrollersconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        CoralRollersUpperMotor.configure(Coralrollersconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    public void spinIn() {
        System.out.println("spinning");
        CoralRollersLowerMotor.set(CoralRollersConstants.maxSpeed);
        CoralRollersUpperMotor.set(CoralRollersConstants.maxSpeed);
    }

    public void spinOut(){
        System.out.println("spinning out");
        CoralRollersLowerMotor.set(-CoralRollersConstants.maxSpeed);
        CoralRollersUpperMotor.set(-CoralRollersConstants.maxSpeed);
    }

    public void stopCoralRollers(){
        CoralRollersLowerMotor.set(0);
        CoralRollersUpperMotor.set(0);
    }
}