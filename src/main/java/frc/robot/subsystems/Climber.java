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

    SparkMax leaderMotor;
    RelativeEncoder leaderEncoder;
    SparkMaxConfig leaderMotorConfig;

    SparkMax followMotor;
    RelativeEncoder followEncoder;
    SparkMaxConfig followMotorConfig;

    public Climber() {
        leaderMotor = new SparkMax(ClimberConstants.leaderMotorID, MotorType.kBrushless);
        leaderMotorConfig = new SparkMaxConfig();
        leaderEncoder = leaderMotor.getEncoder();
        followMotor = new SparkMax(ClimberConstants.followMotorID, MotorType.kBrushless);
        followMotorConfig = new SparkMaxConfig();
        followEncoder = followMotor.getEncoder();
    }

    public void configureDevices() {
        leaderMotorConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        
        followMotorConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .follow(leaderMotor);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Leader Position", leaderEncoder.getPosition());
        SmartDashboard.putNumber("Climber Follower Position", followEncoder.getPosition());
    }

    public Command setClimberSpeed(double speed) {
        return Commands
        .run(
            () -> {
                leaderMotor.set(speed);
                followMotor.set(speed);
            },
            this
        )
        .finallyDo(
            () -> {
                leaderMotor.set(0.0);
                followMotor.set(0.0);
            }
        );
    }
}
