package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
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
    }

    public void configureDevices() {
        digMotorConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        
        nanMotorConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake);

        winchConfig
            .smartCurrentLimit(10)
            .inverted(false)
            .idleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Leader Position", digEncoder.getPosition());
        SmartDashboard.putNumber("Climber Follower Position", nanEncoder.getPosition());
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
            .runOnce(
                () -> {
                    winchMotor.set(speed);;
                },
                this
            );
    }
}
