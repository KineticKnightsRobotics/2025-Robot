package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {



    SparkMax pivotMotor;
    RelativeEncoder pivotEncoder;
    SparkClosedLoopController pivotController;
    SparkMaxConfig pivotMotorConfig;
    

    public Intake() {
        pivotMotor = new SparkMax(0, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotController = pivotMotor.getClosedLoopController();

        configureMotors();
    }

    private void configureMotors() {
        pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig
            .inverted(false)
            .smartCurrentLimit(20);
    }


}
