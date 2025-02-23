package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(1, "rio");
    private final int LedCount = 100;

    private Animation m_toAnimate = null;
;

    public enum AnimationTypes {
        Idle,
        Enabled,
        Intake,
        Outtake,
        HasPiece,
        PieceOut
    }
    private AnimationTypes m_currentAnimation;

    public CANdleSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);

        changeAnimation(AnimationTypes.Idle);
    }

    public void setColors() {
        changeAnimation(AnimationTypes.Idle);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            case Intake:
                m_candle.setLEDs(0,0,255);
                break;
            case Outtake:
                m_candle.setLEDs(255,0,0);
                break;
            case HasPiece:
                m_toAnimate = new StrobeAnimation(0, 0, 255, 0, 25.0/ 256.0, LedCount);
                break;
            case PieceOut:
                m_toAnimate = new StrobeAnimation(255, 0, 0, 0, 25.0/ 256.0, LedCount);
                break;
            case Idle:
                m_toAnimate = new SingleFadeAnimation(0, 255, 0, 0, 0.5, LedCount);
                break;
            case Enabled:
                m_candle.setLEDs(0,255,0);
                break;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(m_toAnimate != null) {
            m_candle.animate(m_toAnimate);
        }
       
       m_candle.modulateVBatOutput(1);
    }

}



