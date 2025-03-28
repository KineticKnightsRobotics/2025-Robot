package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeAffectorConstants;
import frc.robot.Constants.AlgaeAffectorConstants.PivotPositions;

public class algaeAffector extends SubsystemBase {
    
    /**
     * IDs 31, 32, 33 respectively
     * Pivot motors pivot the arm
     * Effector motor drives the end effector
     */
    private SparkMax pivotMotor, rollerMotor;
    private SparkMaxConfig pivotMotorConfig, rollerMotorConfig;
    private AbsoluteEncoder pivotEncoder;
    private SparkClosedLoopController pivotController;

    private DigitalInput proxSensor;

    private double primedPosition;
    private double pivotPosition;

    public algaeAffector() {

        //Config pivot motors
        pivotMotor = new SparkMax(AlgaeAffectorConstants.pivotMotorID, MotorType.kBrushless);
        // Config effector motor
        rollerMotor = new SparkMax(AlgaeAffectorConstants.rollerMotorID, MotorType.kBrushless);
        // Config pivot encoder
        
        proxSensor = new DigitalInput(AlgaeAffectorConstants.proxSensor);
        
        // Apply motor/encoder configs
        configureDevices();
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pivotController = pivotMotor.getClosedLoopController();

        pivotController.setReference(AlgaeAffectorConstants.PivotPositions.home, ControlType.kPosition);
        pivotPosition = PivotPositions.home;
        
    }

    // Set current limits, config motors and encoders
    private void configureDevices() {
        try {
            //pivot motor
            pivotMotorConfig = new SparkMaxConfig();
            pivotMotorConfig
                    .inverted(true)
                    .smartCurrentLimit(30)
                    .closedLoopRampRate(0.0001)
                    .idleMode(IdleMode.kBrake);
            pivotMotorConfig.absoluteEncoder
                .zeroOffset(AlgaeAffectorConstants.encoderOffset)
                .positionConversionFactor(360)
                ;
            pivotMotorConfig
                .closedLoop
                    .p(AlgaeAffectorConstants.PivotPID.P)
                    .i(AlgaeAffectorConstants.PivotPID.I)
                    .d(AlgaeAffectorConstants.PivotPID.D)
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0.0, 360);
            pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


            //Roller motor
            rollerMotorConfig = new SparkMaxConfig();
            rollerMotorConfig
                .inverted(false)
                .smartCurrentLimit(60)
                .closedLoopRampRate(0.0001)
                .idleMode(IdleMode.kBrake);
            rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        } catch (Exception ex) {
            DriverStation.reportError("Failed to configure Algae Subsystem", ex.getStackTrace());
        }

    }

    // Post pivot position and goal to SmartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putNumber("A_Absolute Position", getPivotAbsolutePosition());
        SmartDashboard.putNumber("A_Primed Position", primedPosition);
        SmartDashboard.putBoolean("A_Dignan Algae", hasAlgae());
        SmartDashboard.putBoolean("A_At Position", atPosition());
        SmartDashboard.putData(this);
    }

    // Get the position of the pivotEncoder in degrees
    public double getPivotAbsolutePosition() {
        return pivotEncoder.getPosition();
    }

    public boolean atPosition() {
        return Math.abs(getPivotAbsolutePosition() - pivotPosition) < 5;
    }
    
    public boolean hasAlgae() {
        return proxSensor.get();
    }

    //public Command setPrimedPosition(double position) {
    //    return Commands.runOnce(()-> {primedPosition = position;},this);
    //}

    public Command setAlgaePosition(double position) {
        return Commands.runOnce(()->{pivotController.setReference(position, ControlType.kPosition); pivotPosition = position;});
    }

    public Command intakeAlgae(double intakingPosition, double endingPosition) {
        return Commands
            .runOnce(
                () -> {
                    pivotController.setReference(intakingPosition, ControlType.kPosition);
                    pivotPosition = intakingPosition;
                    rollerMotor.set(-0.05);
                },
                this
            ).andThen(
                Commands.run(
                () -> {
                    rollerMotor.set(-0.8);
                },
                this)
            ).until(() -> hasAlgae())
            .finallyDo(
                () -> {
                    pivotController.setReference(endingPosition, ControlType.kPosition);
                    pivotPosition = endingPosition;
                    rollerMotor.set(-0.25);
                }
            );
    } 

    public Command spitAlgae() {
        return Commands
            .run(
                () -> {
                    rollerMotor.set(1.0);
                },
                this
            )
            .finallyDo(
                () -> {
                    rollerMotor.set(0.0);
                    pivotController.setReference(AlgaeAffectorConstants.PivotPositions.home, ControlType.kPosition);
                    pivotPosition = AlgaeAffectorConstants.PivotPositions.home;
                }
            );
    }
}
