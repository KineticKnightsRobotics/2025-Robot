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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CoralAffectorConstants;

public class coralAffector extends SubsystemBase {
    

    SparkMax rollerMotor;
    SparkMaxConfig rollerMotorConfig;
    DigitalInput beamUpper, beamLower, proxSensor;


    public coralAffector() {
        rollerMotor = new SparkMax(CoralAffectorConstants.coralRollerID, MotorType.kBrushless);
        beamUpper = new DigitalInput(CoralAffectorConstants.beamUpper);
        beamLower = new DigitalInput(CoralAffectorConstants.beamLower);
        proxSensor = new DigitalInput(CoralAffectorConstants.proxSensor);
    }



    public void configDevices() {
        // Effector motor
        rollerMotorConfig = new SparkMaxConfig();
        rollerMotorConfig
            .inverted(true)
            .smartCurrentLimit(30)
            .closedLoopRampRate(0.000001)
            .idleMode(IdleMode.kBrake);

        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Mech Has Coral", hasCoral());
    }

    public boolean hasCoral() {
        return !beamLower.get() || !beamUpper.get();
    }

    public boolean coralLoaded() {
        return !beamLower.get() && beamUpper.get();
    }
    
    public boolean allignedWithPeg() {
        return proxSensor.get();
    }

    // Load a game piece into the robot
    public Command loadCoral() {
        // Set the speed of the affector motor > 0 to run it
        return Commands.run(
            () -> rollerMotor.set(0.3),
            this
        // End condition of linebreak true (piece is in)
        ).until(
            () -> coralLoaded()

        // Once the command is to be finished, stop the affector motor
        ).finallyDo(
            () -> rollerMotor.set(0.0)
        );
    }

    // Spit out the game piece
    public Command spitCoral() {
        // Set the speed of the affector motor > 0 to run it
        return Commands.run(
            () -> rollerMotor.set(1.0)

        // End condition of linebreak true (piece is in)
        ).until(
            () -> !hasCoral()
        // Once the command is to be finished, stop the affector motor
        ).finallyDo(
            () -> rollerMotor.set(0.0)
        );
    }
}
