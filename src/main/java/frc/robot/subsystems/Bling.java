package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import java.util.Random;

public class Bling extends SubsystemBase {

    private final CANdle m_candle;
    private final int LedCount = 100;

    public enum AnimationTypes {
        ColorFlow, Fire, Larson, Rainbow, RgbFade, SingleFade, Strobe,
        Twinkle, TwinkleOff, SetAll, CustomFire
    }

    private Animation m_currentAnimation;
    private AnimationTypes m_currentAnimationType;

    private final int COOLING = 40;
    private final int SPARKING = 150;
    private final int[] heat;
    private final Random rand;

    public Bling() {
        m_candle = new CANdle(1, "rio");

        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.5;
        config.vBatOutputMode = VBatOutputMode.Modulated;

        m_candle.configAllSettings(config, 100);

        heat = new int[LedCount];
        rand = new Random();

        setAnimation(AnimationTypes.SingleFade);
    }

    @Override
    public void periodic() {
        if (m_currentAnimationType == AnimationTypes.CustomFire) {
            runCustomFire();
        } else if (m_currentAnimation != null) {
            m_candle.animate(m_currentAnimation);
        }

        SmartDashboard.putString("Current LED Animation", m_currentAnimationType.name());
    }

    public void setAnimation(AnimationTypes animationType) {
        m_currentAnimationType = animationType;
        switch (animationType) {
            case ColorFlow:
                m_currentAnimation = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, ColorFlowAnimation.Direction.Forward);
                break;
            case Fire:
                m_currentAnimation = new FireAnimation(1, 0.7, LedCount, 1, 0.1);
                break;
            case Larson:
                m_currentAnimation = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, LarsonAnimation.BounceMode.Front, 3);
                break;
            case Rainbow:
                m_currentAnimation = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_currentAnimation = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_currentAnimation = new SingleFadeAnimation(50, 200, 2, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_currentAnimation = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_currentAnimation = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinkleAnimation.TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_currentAnimation = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffAnimation.TwinkleOffPercent.Percent100);
                break;
            case CustomFire:
                m_currentAnimation = null;
                break;
            default:
                m_currentAnimation = null;
                break;
        }
    }

    private void runCustomFire() {
        for (int i = 0; i < LedCount; i++) {
            heat[i] = Math.max(0, heat[i] - rand.nextInt((COOLING * 10 / LedCount) + 2));
        }
        for (int i = LedCount - 1; i >= 2; i--) {
            heat[i] = (heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3;
        }
        if (rand.nextInt(255) < SPARKING) {
            heat[rand.nextInt(7)] = rand.nextInt(95) + 160;
        }
        for (int i = 0; i < LedCount; i++) {
            int g = Math.min(255, heat[i]);
            int b = (int) Math.min(255, heat[i] * 0.5); // Adjust blue component for a cooler effect
            m_candle.setLEDs(0, g, b, 0, i, 1); // Set green and blue values
        }
    }

    public Command setLEDAnimation(AnimationTypes animationType) {
        return Commands.runOnce(() -> setAnimation(animationType), this);
    }

    public Command turnOffLEDs() {
        return Commands.runOnce(() -> m_candle.setLEDs(0, 0, 0, 0, 0, LedCount), this);
    }
}
