package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    SparkMax digMotor;
    RelativeEncoder digEncoder;
    SparkMaxConfig digMotorConfig;

    SparkMax nanMotor;
    RelativeEncoder nanEncoder;
    SparkMaxConfig nanMotorConfig;

    Servo digServo;
    Servo nanServo;



    public Climber() {
        digMotor = new SparkMax(ClimberConstants.digMotorID, MotorType.kBrushless);
        digMotorConfig = new SparkMaxConfig();
        digEncoder = digMotor.getEncoder();
        nanMotor = new SparkMax(ClimberConstants.nanMotorID, MotorType.kBrushless);
        nanMotorConfig = new SparkMaxConfig();
        nanEncoder = nanMotor.getEncoder();

        digServo = new Servo(9);
        nanServo = new Servo(8);

        digServo.set(ClimberConstants.servoRestingDig);
        nanServo.set(ClimberConstants.servoRestingNan);


        configureDevices();
    }

    public void configureDevices() {
        digMotorConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        digMotor.configure(digMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        nanMotorConfig
            .smartCurrentLimit(40)
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        nanMotor.configure(digMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Cl_digPosition", digEncoder.getPosition());
        SmartDashboard.putNumber("Cl_nanPosition", nanEncoder.getPosition());
        SmartDashboard.putNumber("Cl_digServo", digServo.getPosition());
        SmartDashboard.putNumber("Cl_nanServo",nanServo.getPosition());
        SmartDashboard.putData(this);
    }

    public Command setClimberSpeed(double speed) {
        return Commands
        .run(
            () -> {
                digMotor.set(speed);
                nanMotor.set(speed);
            },
            this
        )
        .finallyDo(
            () -> {
                digMotor.set(0.0);
                nanMotor.set(0.0);
            }
        );
    }

    public Command releaseRamp() {
        return Commands
        .runOnce(
            ()-> {
                digServo.set(ClimberConstants.servoReleaseDig);
                nanServo.set(ClimberConstants.servoReleaseNan);
            },
        this
        );
    }

}
