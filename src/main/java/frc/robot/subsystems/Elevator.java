package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
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
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private SparkMax leaderElevatorMotor;
    private SparkMaxConfig leadMotorConfig;
    private SparkMax followElevatorMotor;
    private SparkMaxConfig followMotorConfig;

    private CANcoder elevatorEncoder;

    private ProfiledPIDController elevatorController;

    private double goalPosition;

    public Elevator() {

        //Configure the leader motor
        leaderElevatorMotor = new SparkMax(ElevatorConstants.leaderMotorID, MotorType.kBrushless);
        followElevatorMotor = new SparkMax(ElevatorConstants.followMotorID, MotorType.kBrushless);
        elevatorEncoder = new CANcoder(ElevatorConstants.encoderID);

        elevatorController = new ProfiledPIDController(
            0.001,
            0,
            0,
            new TrapezoidProfile.Constraints(
                1,
                1
            )
        );


        configureMotors();

        elevatorEncoder.setPosition(elevatorEncoder.getAbsolutePosition().getValueAsDouble() - ElevatorConstants.encoderOffset);
        goalPosition = getElevatorPosition(); //Initialize the goal position to wherever we started.
    }

    public void configureMotors() {
        //try {
            leadMotorConfig = new SparkMaxConfig();
            leadMotorConfig
                .inverted(false)                                                                                            //Inverts the motor
                .smartCurrentLimit(60)                                                                                    //Limits # of amps going to the motor
                .closedLoopRampRate(0.01);                                                                                      //Ammount of time for the voltage to ramp i.e it will take 0.01 seconds for the input voltage to go from 1V to 2V
            leaderElevatorMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            followMotorConfig = new SparkMaxConfig();
            followMotorConfig
                .inverted(true)
                .smartCurrentLimit(60)
                .closedLoopRampRate(0.01)
                .follow(ElevatorConstants.leaderMotorID);
            followElevatorMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //}
        //catch (Exception ex){
        //    DriverStation.reportError("Failed to configure Elevator Subsystem", ex.getStackTrace());
        //}
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Goal", getElevatorGoal());
        SmartDashboard.putData(this);
    }

    public double getElevatorPosition() {
        return elevatorEncoder.getPosition().getValueAsDouble() * ElevatorConstants.gearCircumference;
    }
    
    public double getElevatorGoal(){
        return goalPosition;
    }

    public Command setElevatorGoal(double position) {
        return Commands
        .runOnce(
            () -> {
                goalPosition = position;
                elevatorController.setGoal(position);   
            },
            this
        ).unless(
           () ->(position < ElevatorConstants.maxElevatorHeight)
        );
    }
    
    public Command moveElevator() {
        return Commands.run(
            () -> {
                if (getElevatorPosition() >= 0 && getElevatorPosition() < ElevatorConstants.maxElevatorHeight) {
                    leaderElevatorMotor.set(
                        elevatorController.calculate(getElevatorPosition())
                    );
                }
            },
            this
        )
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command zeroElevatorPosition() {
        return Commands.runOnce(
            () -> {elevatorEncoder.setPosition(elevatorEncoder.getAbsolutePosition().getValueAsDouble() - ElevatorConstants.encoderOffset);},
            this);
    }
}
