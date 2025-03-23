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
    private double algaePosition;

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

        goalPosition = ElevatorConstants.Positions.home;
        algaePosition = ElevatorConstants.Positions.deAlgifyL2;
        elevatorEncoder.setPosition(0.0);
    }

    public void configureDevices() {
        try {
            leadMotorConfig = new SparkMaxConfig();
            leadMotorConfig
                .inverted(false)          //Inverts the motor
                .smartCurrentLimit(40)  //Limits # of amps going to the motor
                .idleMode(IdleMode.kCoast)         //Sets idle mode to coast, when the motor is set to 0% output, then it can be freely spun by hand, gravity, etc
                .closedLoopRampRate(0.001);   //Ammount of time in seconds that the motor will take to accellerate from 0% output to 100% output.
            leadMotorConfig
                .encoder
                    .positionConversionFactor(1.0);

            digElevatorMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            nanMotorConfig = new SparkMaxConfig();
            nanMotorConfig
                .inverted(true)         
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kCoast)
                .closedLoopRampRate(0.001);
            
            nanMotorConfig
                .encoder
                    .positionConversionFactor(1.0);


            nanElevatorMotor.configure(nanMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            elevatorEncoderConfig = new CANcoderConfiguration();
            elevatorEncoder.getConfigurator().apply(
                elevatorEncoderConfig.MagnetSensor
                    .withAbsoluteSensorDiscontinuityPoint(1)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                );


        }
        catch (Exception ex){
            DriverStation.reportError("Failed to configure Elevator Subsystem", ex.getStackTrace());
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("E_Position", getElevatorPosition());
        SmartDashboard.putNumber("E_Algae Pos", getElevatorAlgaeGoal());
        SmartDashboard.putNumber("E_Goal", getElevatorGoal());
        SmartDashboard.putBoolean("E_atGoal", elevatorAtGoal());


        SmartDashboard.putNumber("E_AbsolutePosition", elevatorEncoder.getAbsolutePosition().getValueAsDouble());

        SmartDashboard.putNumber("E_digEncoder", digEncoder.getPosition());
        SmartDashboard.putNumber("E_nanEncoder", nanEncoder.getPosition());

        SmartDashboard.putData(this);
    }

    public double getElevatorPosition() {
        return ((elevatorEncoder.getPosition().getValueAsDouble()-ElevatorConstants.encoderOffset) * ElevatorConstants.gearCircumference) + ElevatorConstants.minChassisHeight;
    }
    
    public double getElevatorGoal(){
        return goalPosition;
    }
    public double getElevatorAlgaeGoal() {
        return algaePosition;
    }  

    public boolean elevatorAtGoal() {
        return Math.abs(goalPosition - getElevatorPosition()) < 1.5;
    }
    
    public void setElevatorVoltage(double voltage){
        sysIDVoltage = voltage;
        digElevatorMotor.setVoltage(sysIDVoltage);
    }

    /**
     * Sets the goal position that the elevator will go to the next time elevatorToGoal() is called
     * @param position Elevation in inches from the top of the bottom elevator bar to the bottom of the elevator chassis.
     */
    public Command setElevatorGoal(double position) {
        return Commands
        .runOnce(
            () -> {
                //Clamp new incoming position incase it is ever out of the physical bounds of the elevator.
                goalPosition = MathUtil.clamp(position, ElevatorConstants.minChassisHeight+0.1, ElevatorConstants.maxChassisHeight-0.1);
            },
            this
        );
    }

        /**
     * Sets the goal position that the elevator will go to the next time dealgifying command is run
     * @param position Elevation in inches from the top of the bottom elevator bar to the bottom of the elevator chassis.
     */
    public Command setElevatorDealgify(double position) {
        return Commands
        .runOnce(
            () -> {
                //Clamp new incoming position incase it is ever out of the physical bounds of the elevator.
                algaePosition = MathUtil.clamp(position, ElevatorConstants.minChassisHeight+0.1, ElevatorConstants.maxChassisHeight-0.1);
            },
            this
        );
    }
    

    /**
     * Moves the elvator upwards towards the setpoint using closed loop position control.
     * @return command that does the above
     */
    public Command elevatorToGoal() {
        return Commands
        .runOnce(
            () -> {
                digElevatorMotor.set(0.0); nanElevatorMotor.set(0.0);
            },
            this
        ).andThen(
            Commands.run(
                () -> {
                    if (getElevatorPosition() < ElevatorConstants.maxChassisHeight) {
                        double output = MathUtil.clamp(elevatorController.calculate(getElevatorPosition(), goalPosition),-1.0,1.0);
                        if (getElevatorPosition() < 3) {
                            output = MathUtil.clamp(output, -0.2, 1.0);
                        }
                        if (getElevatorPosition() > 50) {
                            output = MathUtil.clamp(output, -1.0, 0.2);
                        }

                        SmartDashboard.putNumber("E_PID Output", output);
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
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
    }

     /**
     * Moves the elvator upwards towards the setpoint using closed loop position control.
     * @return command that does the above
     */
    public Command elevatorToAlgae() {
        return Commands
        .runOnce(
            () -> {
                digElevatorMotor.set(0.0); nanElevatorMotor.set(0.0);
            },
            this
        ).andThen(
            Commands.run(
                () -> {
                    if (getElevatorPosition() < ElevatorConstants.maxChassisHeight) {
                        double output = MathUtil.clamp(elevatorController.calculate(getElevatorPosition(), algaePosition),-1.0,1.0);
                        if (getElevatorPosition() < 3) {
                            output = MathUtil.clamp(output, -0.2, 1.0);
                        }
                        if (getElevatorPosition() > 50) {
                            output = MathUtil.clamp(output, -1.0, 0.2);
                        }

                        SmartDashboard.putNumber("E_PID Output", output);
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
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
    }   

    /**
     * Moves the elevator to a specific point
     * @return command that does the above.
     */
    public Command elevatorToHeight(double height) {
        return Commands
            .run(
                () -> {
                    double output = MathUtil.clamp(elevatorController.calculate(getElevatorPosition(), height),-0.55,1.0);
                    if (getElevatorPosition() < 20) {
                        output = MathUtil.clamp(output, -0.1, 1.0);
                    }
                    SmartDashboard.putNumber("E_PID Output", output);
                    digElevatorMotor.set(output);
                    nanElevatorMotor.set(output);
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
