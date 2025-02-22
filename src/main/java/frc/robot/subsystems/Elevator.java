package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private SparkMax leaderElevatorMotor;
    private SparkMaxConfig leadMotorConfig;
    private RelativeEncoder leaderEncoder;
    private SparkMax followElevatorMotor;
    private SparkMaxConfig followMotorConfig;
    private RelativeEncoder followEncoder;

    private CANcoder elevatorEncoder;
    private CANcoderConfiguration elevatorEncoderConfig;

    //private ProfiledPIDController elevatorController;
    private PIDController elevatorController;

    private double goalPosition;

    private double sysIDVoltage = 0.0;

    public Elevator() {

        //Configure the leader motor
        leaderElevatorMotor = new SparkMax(ElevatorConstants.leaderMotorID, MotorType.kBrushless);
        leaderEncoder = leaderElevatorMotor.getEncoder();
        followElevatorMotor = new SparkMax(ElevatorConstants.followMotorID, MotorType.kBrushless);
        followEncoder = followElevatorMotor.getEncoder();
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

        goalPosition = ElevatorConstants.ChassisElevationOffset+1;
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
            /*
            leadMotorConfig                                                                                                  //Ammount of time for the voltage to ramp i.e it will take 0.01 seconds for the input voltage to go from 1V to 2V
                .encoder
                    .positionConversionFactor(ElevatorConstants.gearCircumference * ElevatorConstants.gearRatio);
            leadMotorConfig
                .softLimit
                    .reverseSoftLimit(ElevatorConstants.ChassisElevationOffset-1)
                    .forwardSoftLimit(ElevatorConstants.maxChassisHeight+1);
            */
            leaderElevatorMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            followMotorConfig = new SparkMaxConfig();
            followMotorConfig
                .inverted(true)
                .smartCurrentLimit(30)
                .idleMode(IdleMode.kCoast)
                .closedLoopRampRate(0.001);
            
            followMotorConfig
                .encoder
                    .positionConversionFactor(1.0);
            //followMotorConfig
            //    .softLimit
            //        .reverseSoftLimit(ElevatorConstants.maxChassisHeight-1)
            //        .forwardSoftLimit(ElevatorConstants.maxChassisHeight+1);

            followElevatorMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Goal", getElevatorGoal());
        SmartDashboard.putBoolean("Elevator at Goal", elevatorAtGoal());


        SmartDashboard.putNumber("Elevator Absolute Value", elevatorEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Voltage", leaderElevatorMotor.getBusVoltage());

        SmartDashboard.putNumber("Elevator leader encoder", leaderEncoder.getPosition());
        SmartDashboard.putNumber("Elevator follow encoder", followEncoder.getPosition());

        SmartDashboard.putData(this);
    }

    public double getElevatorPosition() {
        //return leaderEncoder.getPosition();
        return ((elevatorEncoder.getPosition().getValueAsDouble()-ElevatorConstants.encoderOffset) * ElevatorConstants.gearCircumference) + ElevatorConstants.ChassisElevationOffset;
    }
    
    public double getElevatorGoal(){
        return goalPosition;
    }

    public boolean elevatorAtGoal() {
        return Math.abs(goalPosition - getElevatorPosition()) < 1.5;
    }
    
    public void setElevatorVoltage(double voltage){
        sysIDVoltage = voltage;
        leaderElevatorMotor.setVoltage(sysIDVoltage);
    }

    public Command setElevatorGoal(double position) {
        return Commands
        .runOnce(
            () -> {
                goalPosition = MathUtil.clamp(position, ElevatorConstants.ChassisElevationOffset+0.1, ElevatorConstants.maxChassisHeight-0.1);
                //elevatorController.setGoal(position);   
            },
            this
        ).unless(
           () ->(position > ElevatorConstants.maxChassisHeight)
        );
    }
    
    public Command moveElevator(BooleanSupplier willElevatorSquish) {
        return Commands
        .runOnce(
            () -> {
                leaderElevatorMotor.set(0.0); followElevatorMotor.set(0.0);
            },
            this
        ).andThen(
            Commands.run(
                () -> {
                    //double newOutput = elevatorController.calculate(getElevatorPosition());
                    SmartDashboard.putBoolean("Will arm Squish", willElevatorSquish.getAsBoolean());
                    if (getElevatorPosition() >= 0 && getElevatorPosition() < ElevatorConstants.maxChassisHeight && willElevatorSquish.getAsBoolean()) {
                        SmartDashboard.putNumber("PID Output",elevatorController.calculate(getElevatorPosition()));

                        double output = MathUtil.clamp(elevatorController.calculate(getElevatorPosition(), goalPosition),-1.0,1.0);

                        if (getElevatorPosition() < 3 || getElevatorPosition() > 50) {
                            output = MathUtil.clamp(output, -0.2, 0.2);
                        }

                        SmartDashboard.putNumber("Elevator output", output);

                        leaderElevatorMotor.set(output);
                        followElevatorMotor.set(output);
                    }
                    else {
                        leaderElevatorMotor.set(0.0);
                        followElevatorMotor.set(0.0);
                    }
                },
                this
            ).until(
                () -> false
            )
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
    }

    public Command setElevatorSpeed(double speed) {
        return Commands.run(
            () -> {
                leaderElevatorMotor.set(speed);
                followElevatorMotor.set(speed);
            }, 
            this)
            .finallyDo(
                () -> {
                    leaderElevatorMotor.set(0.0);
                    followElevatorMotor.set(0.0);
                }
            );
    }

    public Command zeroElevatorPosition() {
        return Commands.runOnce(
            () -> {elevatorEncoder.setPosition(elevatorEncoder.getAbsolutePosition().getValueAsDouble() - ElevatorConstants.encoderOffset);},
            this);
    }
}
