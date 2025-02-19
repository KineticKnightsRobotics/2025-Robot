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

import frc.robot.Constants.EndAffectorConstants;

public class EndAffector extends SubsystemBase {
    

    SparkMax affectorMotor;
    SparkMaxConfig affectorMotorConfig;
    DigitalInput endAffectorBeamBreak;
    DigitalInput endAffectorRangeSensor;


    public EndAffector() {
        affectorMotor = new SparkMax(EndAffectorConstants.affectorMotorID, MotorType.kBrushless);
        endAffectorBeamBreak = new DigitalInput(EndAffectorConstants.beamBreakPort);
        endAffectorRangeSensor = new DigitalInput(EndAffectorConstants.rangeSensorPort);
    }



    public void configDevices() {
                // Effector motor
        affectorMotorConfig = new SparkMaxConfig();
        affectorMotorConfig
            .inverted(true)
            .smartCurrentLimit(30)
            .closedLoopRampRate(0.000001)
            .idleMode(IdleMode.kBrake);

        affectorMotor.configure(affectorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Arm Has Coral", hasCoral());
        SmartDashboard.putBoolean("Arm Has Algae", hasAlgae());
    }

    public boolean hasCoral() {
        return !endAffectorBeamBreak.get();
    }

    public boolean hasAlgae() {
        return endAffectorRangeSensor.get();
    }

    // Load a game piece into the robot
    public Command loadCoral() {
        // Set the speed of the affector motor > 0 to run it
        return Commands.run(
            () -> affectorMotor.set(-0.3),
            this
        // End condition of linebreak true (piece is in)
        ).until(
            () -> hasCoral()

        // Once the command is to be finished, stop the affector motor
        ).finallyDo(
            () -> affectorMotor.set(0.0)
        );
    }
    // Spit out the game piece
    public Command spitCoral() {
        // Set the speed of the affector motor > 0 to run it
        return Commands.run(
            () -> affectorMotor.set(-1.0)

        // End condition of linebreak true (piece is in)
        ).until(
            () -> !hasCoral()
        // Once the command is to be finished, stop the affector motor
        ).finallyDo(
            () -> affectorMotor.set(0.0)
        );
    }

    public Command loadAlgae() {
        return Commands.run(
            () -> affectorMotor.set(0.5),
            this
        ).until(()->hasAlgae())
        .finallyDo(
            () -> {
                affectorMotor.set(0.1);
            }
        );
    }

    public Command spitAlgae() {
        return Commands.run(
            () -> affectorMotor.set(-0.5),
            this
        ).until(()-> !hasAlgae())
        .finallyDo(
            () -> {
                affectorMotor.set(0.0);
            }
        );
    }
}
