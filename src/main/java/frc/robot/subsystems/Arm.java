package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private SparkMax leaderPivotMotor, followPivotMotor, effectorMotor;
    private SparkMaxConfig leadMotorConfig, followMotorConfig, effectorMotorConfig;

    private CANcoder pivotEncoder;
    private CANcoderConfiguration pivotCANcoderConfig;

    private PIDController pivotController;

    private double goalPosition;

    public Arm() {

        //Config pivot motors
        leaderPivotMotor = new SparkMax(ArmConstants.leaderMotorID, MotorType.kBrushless);
        followPivotMotor = new SparkMax(ArmConstants.followMotorID, MotorType.kBrushless);

        // Config effector motor
        effectorMotor = new SparkMax(ArmConstants.effectorMotorID, MotorType.kBrushless);

        // Config pivot encoder
        pivotEncoder = new CANcoder(ArmConstants.encoderID);

        // Config pivot PID
        pivotController = new PIDController(
            ArmConstants.ArmProfiledPID.P,
            ArmConstants.ArmProfiledPID.I, 
            ArmConstants.ArmProfiledPID.D
            );
        
        // Apply motor/encoder configs
        configureDevices();

        // Set goal to idle position
        goalPosition = ArmConstants.idlePosition;

        // Set encoder to 0
        pivotEncoder.setPosition(0.0);
    }

    // Set current limits, config motors and encoders
    private void configureDevices() {
        try {
            // Lead pivot motor
            leadMotorConfig = new SparkMaxConfig();
            leadMotorConfig
                .inverted(false)
                .smartCurrentLimit(30)
                .closedLoopRampRate(0.01);

            leaderPivotMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Follow pivot motor
            followMotorConfig = new SparkMaxConfig();
            followMotorConfig
                .inverted(true)
                .smartCurrentLimit(30)
                .closedLoopRampRate(0.01);

            followPivotMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Effector motor
            effectorMotorConfig = new SparkMaxConfig();
            effectorMotorConfig
                .inverted(false)
                .smartCurrentLimit(30)
                .closedLoopRampRate(0.01);

            effectorMotor.configure(effectorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            // Pivot CANcoder
            pivotCANcoderConfig = new CANcoderConfiguration();
            pivotEncoder.getConfigurator().apply(
                pivotCANcoderConfig.MagnetSensor
                    .withAbsoluteSensorDiscontinuityPoint(1)
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    .withMagnetOffset(-ArmConstants.encoderOffset)
            );

        } catch (Exception ex) {
            DriverStation.reportError("Failed to configure Arm Subsystem", ex.getStackTrace());
        }

    }

    // Post pivot position and goal to SmartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", getPivotEncoderPosition());
        SmartDashboard.putNumber("Pivot Goal", getPivotGoal());

        SmartDashboard.putData(this);
    }

    // Get the position of the pivotEncoder
    public double getPivotEncoderPosition() {
        return pivotEncoder.getPosition().getValueAsDouble();
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
                    if ((getPivotEncoderPosition() < ArmConstants.maxPivotPos) && (getPivotEncoderPosition() > ArmConstants.minPivotPos)) {
                        SmartDashboard.putNumber("Pivot PID Output", pivotController.calculate(getPivotEncoderPosition()));
                        leaderPivotMotor.set(MathUtil.clamp(pivotController.calculate(getPivotEncoderPosition(), goalPosition), -1.0, 1.0));
                        followPivotMotor.set(MathUtil.clamp(pivotController.calculate(getPivotEncoderPosition(), goalPosition), -1.0, 1.0));
                    }
                }, this

            // run until the goal position is met
            ).until(
                () -> false
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
    }

    // Run end effector
    public Command runEffector(double speed) {
        // Set the speed of the effector motor > 0 to run it
        return Commands.runOnce(
            () -> {
                effectorMotor.set(speed);
            }, this

        // Once the command is to be finished, stop the effector motor
        ).finallyDo(
            () -> effectorMotor.set(0.0)
        );
    }

    // Puts the arm to idle position
    public Command idleArm() {
        return setPivotGoal(ArmConstants.idlePosition).andThen(pivotArm());
    }




}
