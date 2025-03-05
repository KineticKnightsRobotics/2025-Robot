package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

    SparkMax winchMotor;
    SparkMaxConfig winchConfig;

    public Climber() {
        digMotor = new SparkMax(ClimberConstants.digMotorID, MotorType.kBrushless);
        digMotorConfig = new SparkMaxConfig();
        digEncoder = digMotor.getEncoder();
        nanMotor = new SparkMax(ClimberConstants.nanMotorID, MotorType.kBrushless);
        nanMotorConfig = new SparkMaxConfig();
        nanEncoder = nanMotor.getEncoder();
        winchMotor = new SparkMax(ClimberConstants.winchID, MotorType.kBrushless);
        winchConfig = new SparkMaxConfig();
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

        winchConfig
            .smartCurrentLimit(10)
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        winchMotor.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("C_digPosition", digEncoder.getPosition());
        SmartDashboard.putNumber("C_nanPosition", nanEncoder.getPosition());
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

    public Command setWinchSpeed(double speed) {
        return Commands
            .run(
                () -> {
                    winchMotor.set(speed);;
                },
                this
            ).finallyDo(
                () -> {
                    winchMotor.set(0.0);
                }
            );
    }
}
