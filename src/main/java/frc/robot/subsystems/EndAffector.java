package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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


    public EndAffector() {
        affectorMotor = new SparkMax(EndAffectorConstants.affectorMotorID, MotorType.kBrushless);
        endAffectorBeamBreak = new DigitalInput(EndAffectorConstants.beamBreakPort);
    }



    public void configDevices() {
                // Effector motor
        affectorMotorConfig = new SparkMaxConfig();
        affectorMotorConfig
            .inverted(true)
            .smartCurrentLimit(30)
            .closedLoopRampRate(0.01);
        affectorMotor.configure(affectorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Arm Has Gamepiece", hasGamepiece());
    }

    public boolean hasGamepiece() {
        return !endAffectorBeamBreak.get();
    }

    // Load a game piece into the robot
    public Command loadGamePiece(double speed) {
        // Set the speed of the affector motor > 0 to run it
        return Commands.run(
            () -> affectorMotor.set(speed),
            this
        // End condition of linebreak true (piece is in)
        ).until(
            () -> hasGamepiece()

        // Once the command is to be finished, stop the affector motor
        ).finallyDo(
            () -> affectorMotor.set(0.0)
        );
    }
    // Spit out the game piece
    public Command spitThatShitOut(double speed) {
        // Set the speed of the affector motor > 0 to run it
        return Commands.run(
            () -> affectorMotor.set(speed)

        // End condition of linebreak true (piece is in)
        ).until(
            () -> !hasGamepiece()
        // Once the command is to be finished, stop the affector motor
        ).finallyDo(
            () -> affectorMotor.set(0.0)
        );
    }
}
