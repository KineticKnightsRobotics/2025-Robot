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

    private final CANdle m_candleLeft;
    private final CANdle m_candleRight;
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
        m_candleRight = new CANdle(1, "rio");
        m_candleLeft = new CANdle(0, "rio");


        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.5;
        config.vBatOutputMode = VBatOutputMode.Modulated;

        m_candleRight.configAllSettings(config, 100);
        m_candleLeft.configAllSettings(config, 100);

        heat = new int[LedCount];
        rand = new Random();

        setAnimation(AnimationTypes.SingleFade);
    }

    @Override
    public void periodic() {
        if (m_currentAnimationType == AnimationTypes.CustomFire) {
            runCustomFire();
        } else if (m_currentAnimation != null) {
            m_candleRight.animate(m_currentAnimation);
            m_candleLeft.animate(m_currentAnimation);

        }

        SmartDashboard.putString("B_Current LED Animation", m_currentAnimationType.name());
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
                // Initialize heat values to zero for clean start
                for (int i = 0; i < LedCount; i++) {
                    heat[i] = 0;
                }
                break;
            default:
                m_currentAnimation = null;
                break;
        }
    }

    private void runCustomFire() {
        // Cool down every cell a little
        for (int i = 0; i < LedCount; i++) {
            int cooling = (COOLING * 10 / LedCount) + 2;
            cooling = Math.max(1, cooling); // Ensure cooling is at least 1 to avoid nextInt(0)
            heat[i] = Math.max(0, heat[i] - rand.nextInt(cooling));
        }
        
        // Heat from each cell drifts up and diffuses
        for (int i = LedCount - 1; i >= 2; i--) {
            heat[i] = (heat[i - 1] + heat[i - 2] + heat[i]) / 3;
        }
        
        // Randomly ignite new sparks near the bottom
        if (rand.nextInt(255) < SPARKING) {
            int sparkPos = rand.nextInt(Math.min(7, LedCount));
            int sparkHeat = rand.nextInt(95) + 160;
            heat[sparkPos] = Math.min(255, heat[sparkPos] + sparkHeat);
        }
        
        // Map from heat to LED colors - display in reverse to simulate upward flame movement
        for (int i = 0; i < LedCount; i++) {
            int heatValue = MathUtil.clamp(heat[i], 0, 255);
            // More realistic fire colors: red dominant, less green, minimal blue
            int r = heatValue;
            int g = (int)(heatValue * 0.3);
            int b = (int)(heatValue * 0.1);
            
            // Display the fire upside down (optional - more natural flame movement)
            int displayPos = LedCount - 1 - i;
            if (displayPos >= 0 && displayPos < LedCount) {
                m_candleRight.setLEDs(r, g, b, 0, displayPos, 1);
                m_candleLeft.setLEDs(r, g, b, 0, displayPos, 1);
            }
        }
    }

    public Command setLEDAnimation(AnimationTypes animationType) {
        return Commands.runOnce(() -> setAnimation(animationType), this);
    }

    public Command turnOffLEDs() {
        return Commands.runOnce(() -> m_candleLeft.setLEDs(0, 0, 0, 0, 0, LedCount)).andThen(()->m_candleRight.setLEDs(0,0,0,0,0,LedCount));
    }
}
