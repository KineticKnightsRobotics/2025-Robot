package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private SparkMax digElevatorMotor;
    private SparkMaxConfig leadMotorConfig;
    private RelativeEncoder digEncoder;
    private SparkMax nanElevatorMotor;
    private SparkMaxConfig nanMotorConfig;
    private RelativeEncoder nanEncoder;

    private CANcoder elevatorEncoder;
    private CANcoderConfiguration elevatorEncoderConfig;

    //private ProfiledPIDController elevatorController;
    private PIDController elevatorController;

    private double goalPosition;

    private double sysIDVoltage = 0.0;

    public Elevator() {

        //Configure the dig motor
        digElevatorMotor = new SparkMax(ElevatorConstants.digMotorID, MotorType.kBrushless);
        digEncoder = digElevatorMotor.getEncoder();
        nanElevatorMotor = new SparkMax(ElevatorConstants.nanMotorID, MotorType.kBrushless);
        nanEncoder = nanElevatorMotor.getEncoder();
        elevatorEncoder = new CANcoder(ElevatorConstants.encoderID);


        elevatorController = new PIDController(
            ElevatorConstants.ElevatorProfiledPID.P,
            ElevatorConstants.ElevatorProfiledPID.I,
            ElevatorConstants.ElevatorProfiledPID.D
            );


        /*elevatorController = new ProfiledPIDController(
            10.0,
            0,
            0.00,
            new TrapezoidProfile.Constraints(
                0.3,
                0.1
            )
        ); */

        configureDevices();

        goalPosition = ElevatorConstants.chassisHome+1;
        elevatorEncoder.setPosition(0.0);
    }

    public void configureDevices() {
        try {
            leadMotorConfig = new SparkMaxConfig();
            leadMotorConfig
                .inverted(false)                                                                                            //Inverts the motor
                .smartCurrentLimit(30)     
                .idleMode(IdleMode.kCoast)                                                                               //Limits # of amps going to the motor
                .closedLoopRampRate(0.001);
            leadMotorConfig
                .encoder
                .positionConversionFactor(1.0/*ElevatorConstants.gearCircumference * ElevatorConstants.gearRatio / 2*/);

            digElevatorMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            nanMotorConfig = new SparkMaxConfig();
            nanMotorConfig
                .inverted(true)
                .smartCurrentLimit(30)
                .idleMode(IdleMode.kCoast)
                .closedLoopRampRate(0.001);
            
            nanMotorConfig
                .encoder
                    .positionConversionFactor(1.0);
            //nanMotorConfig
            //    .softLimit
            //        .reverseSoftLimit(ElevatorConstants.maxChassisHeight-1)
            //        .forwardSoftLimit(ElevatorConstants.maxChassisHeight+1);

            nanElevatorMotor.configure(nanMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            elevatorEncoderConfig = new CANcoderConfiguration();
            elevatorEncoder.getConfigurator().apply(
                elevatorEncoderConfig.MagnetSensor
                    .withAbsoluteSensorDiscontinuityPoint(1)
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    //.withMagnetOffset(0.0)
                );


        }
        catch (Exception ex){
            DriverStation.reportError("Failed to configure Elevator Subsystem", ex.getStackTrace());
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("E_Position", getElevatorPosition());
        SmartDashboard.putNumber("E_Goal", getElevatorGoal());
        SmartDashboard.putBoolean("E_atGoal", elevatorAtGoal());


        SmartDashboard.putNumber("E_AbsolutePosition", elevatorEncoder.getAbsolutePosition().getValueAsDouble());

        SmartDashboard.putNumber("E_digEncoder", digEncoder.getPosition());
        SmartDashboard.putNumber("E_nanEncoder", nanEncoder.getPosition());

        SmartDashboard.putData(this);
    }

    public double getElevatorPosition() {
        //return digEncoder.getPosition();
        return ((elevatorEncoder.getPosition().getValueAsDouble()-ElevatorConstants.encoderOffset) * ElevatorConstants.gearCircumference) + ElevatorConstants.chassisHome;
    }
    
    public double getElevatorGoal(){
        return goalPosition;
    }

    public boolean elevatorAtGoal() {
        return Math.abs(goalPosition - getElevatorPosition()) < 1.5;
    }
    
    public void setElevatorVoltage(double voltage){
        sysIDVoltage = voltage;
        digElevatorMotor.setVoltage(sysIDVoltage);
    }

    public Command setElevatorGoal(double position) {
        return Commands
        .runOnce(
            () -> {
                goalPosition = MathUtil.clamp(position, ElevatorConstants.chassisHome+0.1, ElevatorConstants.maxChassisHeight-0.1);
                //elevatorController.setGoal(position);   
            },
            this
        ).unless(
           () ->(position > ElevatorConstants.maxChassisHeight)
        );
    }
    
    public Command moveElevator() {
        return Commands
        .runOnce(
            () -> {
                digElevatorMotor.set(0.0); nanElevatorMotor.set(0.0);
            },
            this
        ).andThen(
            Commands.run(
                () -> {
                    //double newOutput = elevatorController.calculate(getElevatorPosition());
                    if (getElevatorPosition() < ElevatorConstants.maxChassisHeight) {
                        SmartDashboard.putNumber("PID Output",elevatorController.calculate(getElevatorPosition()));

                        double output = MathUtil.clamp(elevatorController.calculate(getElevatorPosition(), goalPosition),-1.0,1.0);

                        if (getElevatorPosition() < 3 || getElevatorPosition() > 50) {
                            output = MathUtil.clamp(output, -0.2, 0.2);
                        }

                        SmartDashboard.putNumber("Elevator output", output);

                        digElevatorMotor.set(output);
                        nanElevatorMotor.set(output);
                    }
                    else {
                        digElevatorMotor.set(0.0);
                        nanElevatorMotor.set(0.0);
                    }
                },
                this
                )
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
    }

    public Command homeElevator() {
        return Commands
        .runOnce(
            () -> {
                digElevatorMotor.set(0.0); nanElevatorMotor.set(0.0);
            },
            this
        ).andThen(
            Commands.run(
                () -> {
                    //double newOutput = elevatorController.calculate(getElevatorPosition());
                    if (getElevatorPosition() < ElevatorConstants.maxChassisHeight) {
                        double output = MathUtil.clamp(elevatorController.calculate(getElevatorPosition(), ElevatorConstants.Positions.home),-1.0,1.0);

                        if (getElevatorPosition() < 3 || getElevatorPosition() > 50) {
                            output = MathUtil.clamp(output, -0.2, 0.2);
                        }

                        //SmartDashboard.putNumber("Elevator output", output);

                        digElevatorMotor.set(output);
                        nanElevatorMotor.set(output);
                    }
                    else {
                        digElevatorMotor.set(0.0);
                        nanElevatorMotor.set(0.0);
                    }
                },
                this
                )
                .until(() -> elevatorAtGoal())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
    }

    public Command setElevatorSpeed(double speed) {
        return Commands.run(
            () -> {
                digElevatorMotor.set(speed);
                nanElevatorMotor.set(speed);
            }, 
            this)
            .finallyDo(
                () -> {
                    digElevatorMotor.set(0.0);
                    nanElevatorMotor.set(0.0);
                }
            );
    }

    public Command zeroElevatorPosition() {
        return Commands.runOnce(
            () -> {elevatorEncoder.setPosition(elevatorEncoder.getAbsolutePosition().getValueAsDouble() - ElevatorConstants.encoderOffset);},
            this);
    }
}
