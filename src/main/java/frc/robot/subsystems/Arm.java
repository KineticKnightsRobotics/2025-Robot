package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    
    /**
     * IDs 31, 32, 33 respectively
     * Pivot motors pivot the arm
     * Effector motor drives the end effector
     */
    private SparkMax leaderPivotMotor, followPivotMotor, affectorMotor;
    private SparkMaxConfig leadMotorConfig, followMotorConfig, affectorMotorConfig;

    private CANcoder pivotEncoder;
    private CANcoderConfiguration pivotCANcoderConfig;

    private PIDController pivotController;

    private DigitalInput endAffectorBeamBreak;

    private double goalPosition;

    public Arm() {

        //Config pivot motors
        leaderPivotMotor = new SparkMax(ArmConstants.leaderMotorID, MotorType.kBrushless);
        followPivotMotor = new SparkMax(ArmConstants.followMotorID, MotorType.kBrushless);

        // Config effector motor

        // Config pivot encoder
        pivotEncoder = new CANcoder(ArmConstants.encoderID);

        // Config pivot PID
        pivotController = new PIDController(
            ArmConstants.ArmProfiledPID.P,
            ArmConstants.ArmProfiledPID.I, 
            ArmConstants.ArmProfiledPID.D
            );
        pivotController.enableContinuousInput(0, 360);

        
        // Apply motor/encoder configs
        configureDevices();

        // Set goal to idle position
        goalPosition = getPivotEncoderPosition();

        // Set encoder to 0
        //pivotEncoder.setPosition(pivotEncoder.getAbsolutePosition().getValueAsDouble());// - ArmConstants.encoderOffset);
    }

    // Set current limits, config motors and encoders
    private void configureDevices() {
        try {
            // Lead pivot motor
            leadMotorConfig = new SparkMaxConfig();
            leadMotorConfig
                .inverted(true)
                .smartCurrentLimit(30)
                .closedLoopRampRate(1)
                .idleMode(IdleMode.kBrake);

            leaderPivotMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            // Follow pivot motor
            followMotorConfig = new SparkMaxConfig();
            followMotorConfig
                .inverted(false)
                .smartCurrentLimit(30)
                .closedLoopRampRate(1)
                .idleMode(IdleMode.kBrake);

            followPivotMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            
            // Pivot CANcoder
            pivotCANcoderConfig = new CANcoderConfiguration();
            pivotEncoder.getConfigurator().apply(
                pivotCANcoderConfig.MagnetSensor
                    .withAbsoluteSensorDiscontinuityPoint(1)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withMagnetOffset(-ArmConstants.encoderOffset)
            );

        } catch (Exception ex) {
            DriverStation.reportError("Failed to configure Arm Subsystem", ex.getStackTrace());
        }

    }

    // Post pivot position and goal to SmartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Absolute Poistion", pivotEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm Pivot Position", getPivotEncoderPosition());
        SmartDashboard.putNumber("Arm Pivot Goal", getPivotGoal());

        SmartDashboard.putData(this);
    }

    // Get the position of the pivotEncoder in degrees
    public double getPivotEncoderPosition() {
        return (pivotEncoder.getAbsolutePosition().getValueAsDouble() /*- ArmConstants.encoderOffset*/) * 360;
    }

    // Get the pivot goal of the PID
    public double getPivotGoal() {
        return goalPosition;
    }

    // Set the goal of the pivot
    public Command setPivotGoal(double position) {
        return Commands
        .runOnce(
            () -> {
                // Set the goalPosition of the PID to the passed position value
                goalPosition = MathUtil.clamp(position, ArmConstants.minPivotPos, ArmConstants.maxPivotPos);
            }, this
        
        // unless the passed position value is past the min or max bounds
        ).unless(
            () -> (position > ArmConstants.maxPivotPos) || (position < ArmConstants.minPivotPos)
        );
    }

    // Pivot the arm to the pivot goal
    public Command pivotArm() {
        return Commands
        .runOnce(
            () -> {
                // Set the speed of the pivot motors to 0.0
                leaderPivotMotor.set(0.0);
                followPivotMotor.set(0.0);
            }, this

        // Once the motors are stopped, move to the goal position with PID
        ).andThen(
            Commands.run(
                () -> {
                    // If the encoder position is within the rotation bounds, pivot the arm to the goal
                    if (/*(getPivotEncoderPosition() < ArmConstants.maxPivotPos) &&*/(getPivotEncoderPosition() > ArmConstants.minPivotPos)) {
                        //SmartDashboard.putNumber("Pivot PID Output", pivotController.calculate(getPivotEncoderPosition()));
                        leaderPivotMotor.set(MathUtil.clamp(pivotController.calculate(getPivotEncoderPosition(), goalPosition), -1.0, 1.0));
                        followPivotMotor.set(MathUtil.clamp(pivotController.calculate(getPivotEncoderPosition(), goalPosition), -1.0, 1.0));
                    }
                    else {
                        leaderPivotMotor.set(0.0);
                        followPivotMotor.set(0.0);
                    }
                }, this
            // run until the goal position is met
            ).until(
                () -> Math.abs(getPivotEncoderPosition() - goalPosition) < 0.5
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
    }

}
