package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeAffectorConstants;

public class algaeAffector extends SubsystemBase {
    
    /**
     * IDs 31, 32, 33 respectively
     * Pivot motors pivot the arm
     * Effector motor drives the end effector
     */
    private SparkMax pivotMotor, rollerMotor;
    private SparkMaxConfig pivotMotorConfig, rollerMotorConfig;
    private RelativeEncoder pivotEncoder;
    private CANcoder pivotAbsoluteEncoder;

    private CANcoderConfiguration pivotCANcoderConfig;
    private SparkClosedLoopController pivotController;

    private DigitalInput proxSensor;

    private double goalPosition;

    public algaeAffector() {

        //Config pivot motors
        pivotMotor = new SparkMax(AlgaeAffectorConstants.pivotMotorID, MotorType.kBrushless);
        // Config effector motor
        rollerMotor = new SparkMax(AlgaeAffectorConstants.rollerMotorID, MotorType.kBrushless);
        // Config pivot encoder
        pivotAbsoluteEncoder = new CANcoder(AlgaeAffectorConstants.absoluteEncoderID);
        
        proxSensor = new DigitalInput(AlgaeAffectorConstants.proxSensor);
        // Apply motor/encoder configs
        configureDevices();
    }

    // Set current limits, config motors and encoders
    private void configureDevices() {
        try {
            //pivot motor
            pivotMotorConfig = new SparkMaxConfig();
            pivotMotorConfig
                    .inverted(true)
                    .smartCurrentLimit(30)
                    .closedLoopRampRate(1)
                    .idleMode(IdleMode.kBrake);
            pivotMotorConfig
                .encoder
                    .positionConversionFactor(360 * AlgaeAffectorConstants.algaePivotGearRatio)
                    .velocityConversionFactor(360 * AlgaeAffectorConstants.algaePivotGearRatio)
                    .inverted(false);
            pivotMotorConfig
                .closedLoop
                    .p(AlgaeAffectorConstants.PivotPID.P)
                    .i(AlgaeAffectorConstants.PivotPID.I)
                    .d(AlgaeAffectorConstants.PivotPID.D);
            pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            pivotEncoder = pivotMotor.getEncoder();




            //Roller motor
            rollerMotorConfig = new SparkMaxConfig();
            rollerMotorConfig
                .inverted(false)
                .smartCurrentLimit(30)
                .closedLoopRampRate(1)
                .idleMode(IdleMode.kBrake);
            rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            
            // Pivot CANcoder
            pivotCANcoderConfig = new CANcoderConfiguration();
            pivotAbsoluteEncoder.getConfigurator().apply(
                pivotCANcoderConfig.MagnetSensor
                    .withAbsoluteSensorDiscontinuityPoint(1)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withMagnetOffset(-AlgaeAffectorConstants.encoderOffset)
            );

        } catch (Exception ex) {
            DriverStation.reportError("Failed to configure Arm Subsystem", ex.getStackTrace());
        }

    }

    // Post pivot position and goal to SmartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Mech Absolute Position", getPivotAbsolutePosition());
        SmartDashboard.putNumber("Algae Mech Pivot Goal", getPivotGoal());
        SmartDashboard.putData(this);
    }

    // Get the position of the pivotEncoder in degrees
    public double getPivotAbsolutePosition() {
        return (pivotAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() /*- ArmConstants.encoderOffset*/) * 360;
    }

    // Get the pivot goal of the PID
    public double getPivotGoal() {
        return goalPosition;
    }
    
    public boolean hasAlgae() {
        return proxSensor.get();
    }

    private void setPivotGoal(double position) {
        // Set the goalPosition of the PID to the passed position value
        pivotController.setReference(position, ControlType.kPosition);
        goalPosition = position;
    }   

    // Set the goal of the pivot
    public Command setPivotPosition(double position) {
        return Commands
        .runOnce(
            () -> {
                setPivotGoal(position);
            }, this
        
        // unless the passed position value is past the min or max bounds
        );
    }

    public Command captureAlgae(double position) {
        return Commands
            .runOnce(
                () -> {
                    setPivotPosition(position);
                    rollerMotor.set(0.0);
                },
                this
            ).andThen(
                () -> {
                    rollerMotor.set(-0.5);
                },
                this
            ).until(() -> hasAlgae())
            .finallyDo(
                () -> {
                    setPivotPosition(AlgaeAffectorConstants.PivotPositions.home);
                    rollerMotor.set(0.5);
                }
            );
    } 

    public Command scoreProcessor() {
        return Commands
            .runOnce(
                () -> {
                    setPivotGoal(AlgaeAffectorConstants.PivotPositions.home);
                    rollerMotor.set(0.0);
                },
                this
            ).andThen(
                () -> {
                    rollerMotor.set(0.5);
                }
            ).until(() -> !hasAlgae())
            .finallyDo(
                () -> {
                    rollerMotor.set(0.0);
                }
            );
    }
}
