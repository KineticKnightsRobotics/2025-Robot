package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Constants.CoralAffectorConstants;

public class coralAffector extends SubsystemBase {
    

    SparkMax rollerMotor, rampMotor;
    SparkMaxConfig rollerMotorConfig, rampMotorConfig;
    DigitalInput beamUpper, beamLower, proxSensor, rampSensor;
    //AnalogInput ultrasonicSensor;
    Ultrasonic ultrasonicSensor;

    //double voltage_scale_factor = 5/RobotController.getVoltage5V();
    
    private Debouncer debouncer = new Debouncer(0.04,DebounceType.kRising);

    public boolean dDribble = false;


    public coralAffector() {
        rollerMotor = new SparkMax(CoralAffectorConstants.coralRollerID, MotorType.kBrushless);
        rampMotor = new SparkMax(CoralAffectorConstants.coralRampID, MotorType.kBrushless);
        beamUpper = new DigitalInput(CoralAffectorConstants.beamUpper);
        beamLower = new DigitalInput(CoralAffectorConstants.beamLower);
        proxSensor = new DigitalInput(CoralAffectorConstants.proxSensor);
        rampSensor = new DigitalInput(CoralAffectorConstants.rampSensor);
        //ultrasonicSensor = new Ultrasonic(1,2);
        //ultrasonicSensor.setAutomaticMode(true);
        //ultrasonicSensor.
        configDevices();
    }



    public void configDevices() {
        // Effector motor
        rollerMotorConfig = new SparkMaxConfig();
        rollerMotorConfig
            .inverted(false)
            .smartCurrentLimit(30)
            .closedLoopRampRate(0.000001)
            .idleMode(IdleMode.kBrake);
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rampMotorConfig = new SparkMaxConfig();
        rampMotorConfig
            .inverted(false)
            .smartCurrentLimit(10)
            .closedLoopRampRate(0.00001)
            .idleMode(IdleMode.kBrake);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("C_Dignan Coral", hasCoral());
        SmartDashboard.putBoolean("C_Coral Upper Beambreak", !beamUpper.get());
        SmartDashboard.putBoolean("C_Coral Lower Beambreak", !beamLower.get());
        SmartDashboard.putBoolean("C_AllignedWithPeg", allignedWithPeg());
        SmartDashboard.putBoolean("C_Coral on Ramp", rampHasCoral());

        //SmartDashboard.putNumber("C_Ultrasensor Output", ultrasonicSensor.getRangeInches());
        SmartDashboard.putData(this);

        //ultrasonicSensor.ping();
    }

    /**
     * @return True when beam is OBSTRUCTED
     */
    public boolean entranceBeambreak() {
        return !beamUpper.get();
    }
    /**
     * @return True when beam is OBSTRUCTED
     */
    public boolean exitBeambreak() {
        return !beamLower.get();
    }

    /**
     * @return True when EITHER beambreak is obstructed
     */
    public boolean hasCoral() {
        return entranceBeambreak() || exitBeambreak();
    }

    //public double getVoltageScaleFactor() {
   // }
    
    public boolean allignedWithPeg() {
        return debouncer.calculate(proxSensor.get());
        //return proxSensor.get();
        //in inches
        //return ((ultrasonicSensor.getRangeInches() > 12 && ultrasonicSensor.getRangeInches()< 22));
    }

    public boolean rampHasCoral() {
        return rampSensor.get();
    }


    public Command loadCoral() {
        return new SequentialCommandGroup(
            Commands.run(
                () -> {rollerMotor.set(0.8);rampMotor.set(1.0);},
                this).until(()->entranceBeambreak()),
            Commands.run(
                () -> {rollerMotor.set(0.2);},
                this).until(()->(!entranceBeambreak() && exitBeambreak())),
            Commands.run(
                () -> {rollerMotor.set(-0.2);},
                this).until(()->entranceBeambreak())
        ).finallyDo(
            () -> {rollerMotor.set(0.0);rampMotor.set(0.0);}
        );
    }

    public Command loadCoralAuto() {
        return new SequentialCommandGroup(
            Commands.run(
                () -> {rollerMotor.set(0.8);},
                this).until(()->entranceBeambreak()),
            Commands.run(
                () -> {rollerMotor.set(0.2);},
                this).until(()->(!entranceBeambreak() && exitBeambreak())),
            Commands.run(
                () -> {rollerMotor.set(-0.2);},
                this).until(()->entranceBeambreak())
        ).finallyDo(
            () -> {rollerMotor.set(0.0);}
        );
    }

    public Command setRampSpeed(double speed) {
        return Commands.runOnce(()-> rampMotor.set(speed));
    }

    /*
    // Load a game piece into the robot
    public Command loadCoral() {
        return Commands.run(
            () -> rollerMotor.set(0.2),
            this
        ).until(() -> (!entranceBeambreak() && exitBeambreak())
        ).finallyDo(
            () -> rollerMotor.set(0.0)
        );
    }
    */

    // Spit out the game piece
    public Command spitCoral() {
        // Set the speed of the affector motor > 0 to run it
        return Commands.run(
            () -> {rollerMotor.set((dDribble? 0.1 : 0.50));}
        // End condition of linebreak true (piece is in)
        ).until(
            () -> !hasCoral()
        // Once the command is to be finished, stop the affector motor
        ).finallyDo(
            () -> {rollerMotor.set(0.0);}
        );
    }
}
