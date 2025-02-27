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

public class CANdleSubsystem extends SubsystemBase {

    private final CANdle m_candle;
    private final int LedCount = 100;

    public enum AnimationTypes {
        ColorFlow, Fire, Larson, Rainbow, RgbFade, SingleFade, Strobe,
        Twinkle, TwinkleOff, SetAll, CustomFire
    }

    private Animation m_currentAnimation;
    private AnimationTypes m_currentAnimationType;

    private int COOLING = 40;
    private int SPARKING = 150;
    private final int[] heat;
    private final Random rand;

    public CANdleSubsystem() {
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
        // Cool down every cell a little
        for (int i = 0; i < LedCount; i++) {
            heat[i] = Math.max(0, heat[i] - rand.nextInt((COOLING * 10 / LedCount) + 2));
        }
        
        // Heat propagation from bottom to top (assuming LED 0 is physically at bottom)
        // First handle special case for bottom LEDs
        if (LedCount >= 3) {
            // Handle LED 0 separately (sources heat from nowhere/randomly)
            if (rand.nextInt(255) < SPARKING) {
                heat[0] = Math.min(255, heat[0] + rand.nextInt(160) + 90);
            }
            
            // Heat propagation - bottom to top
            for (int i = LedCount - 1; i >= 1; i--) {
                // Each LED gets heat from the ones below it
                heat[i] = (heat[i] + heat[Math.max(0, i-1)] * 2) / 3;
            }
        }
        
        // Randomly ignite new sparks near the bottom
        if (rand.nextInt(255) < SPARKING) {
            int y = rand.nextInt(Math.min(7, LedCount));
            heat[y] = Math.min(255, heat[y] + rand.nextInt(95) + 160);
        }
        
        // Convert heat to LED colors
        for (int i = 0; i < LedCount; i++) {
            // Map heat to colors - improved green fire palette
            int heatValue = Math.min(255, heat[i]);
            
            // Cooler parts: more blue, less green (blue-green)
            // Hotter parts: more green, touch of red (yellowish-green)
            int r = (int)(heatValue * 0.3); // A bit more red for realism
            int g = heatValue;
            int b = (int)(heatValue * 0.5 * (1.0 - (heatValue / 255.0))); // More blue in cooler areas
            
            // Clamp values
            r = Math.min(255, Math.max(0, r));
            g = Math.min(255, Math.max(0, g));
            b = Math.min(255, Math.max(0, b));
            
            m_candle.setLEDs(r, g, b, 0, i, 1);
        }
    }
    
    // Add method to adjust fire parameters
    public void setFireParameters(int cooling, int sparking) {
        this.COOLING = MathUtil.clamp(cooling, 0, 255);
        this.SPARKING = MathUtil.clamp(sparking, 0, 255);
    }
    
    // Add command to adjust fire parameters
    public Command createSetFireParametersCommand(int cooling, int sparking) {
        return Commands.runOnce(() -> {
            this.COOLING = MathUtil.clamp(cooling, 0, 255);
            this.SPARKING = MathUtil.clamp(sparking, 0, 255);
        }, this);
    }

    public Command setLEDAnimation(AnimationTypes animationType) {
        return Commands.runOnce(() -> setAnimation(animationType), this);
    }

    public Command turnOffLEDs() {
        return Commands.runOnce(() -> m_candle.setLEDs(0, 0, 0, 0, 0, LedCount), this);
    }
}
