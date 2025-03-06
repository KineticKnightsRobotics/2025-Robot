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
        Twinkle, TwinkleOff, SetAll, CustomFire, CoralPulse, AlgaePulse,ThirtySeconds
    }

    private Animation m_currentAnimation;
    private AnimationTypes m_currentAnimationType;

    private final int COOLING = 55;  // Increased cooling for more dramatic valleys
    private final int SPARKING = 180; // Increased sparking for more intense flames
    private final int[] heat;
    private final Random rand;

    private int frameCounter = 0;
    private final int FRAME_SKIP = 5;  // Only update every 5 frames
    private boolean[] ledChanged;

    private int pulseCounter = 0;
    private final int PULSE_PERIOD = 50; // Controls the flash speed

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
        ledChanged = new boolean[LedCount];
        rand = new Random();

        setAnimation(AnimationTypes.SingleFade);
    }

    @Override
    public void periodic() {
        frameCounter++;
        pulseCounter++;
        
        // Use built-in animations rather than custom ones
        if (m_currentAnimation != null) {
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
                m_currentAnimation = new RainbowAnimation(0.5, 0.1, LedCount);
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
                // Switch to use built-in fire animation instead of custom
                
                m_currentAnimation = null;
                // Clear animations and LEDs
                m_candleRight.clearAnimation(0);
                m_candleLeft.clearAnimation(0);
                m_candleRight.setLEDs(0, 0, 0, 0, 0, LedCount);
                m_candleLeft.setLEDs(0, 0, 0, 0, 0, LedCount);
                
                // Initialize heat values to zero for clean start
                for (int i = 0; i < LedCount; i++) {
                    heat[i] = 0;
                    ledChanged[i] = true; // Mark all LEDs for initial update
                }
                // Switch to use built-in fire animation instead of custom
                break;
                
            case CoralPulse:
                // Purple strobe for coral
                m_currentAnimation = new StrobeAnimation(200, 0, 255, 0, 0.4, LedCount);
                break;
                
            case AlgaePulse:
                // Green strobe for algae
                m_currentAnimation = new StrobeAnimation(0, 255, 0, 0, 0.4, LedCount);
                break;
                case ThirtySeconds:
                m_currentAnimation = new StrobeAnimation(255, 0, 0, 0, 98.0 / 256.0, LedCount);

                
            default:
                m_currentAnimation = null;
                break;
        }
    }

    // Keep all custom animation methods for potential future use
    private void runCustomFire() {
        try {
            // Store previous heat values to detect changes
            int[] prevHeat = new int[LedCount];
            System.arraycopy(heat, 0, prevHeat, 0, LedCount);
            
            // Cool down every cell a little
            for (int i = 0; i < LedCount; i++) {
                int cooling = (COOLING * 10 / LedCount) + 2;
                cooling = Math.max(1, cooling); // Ensure cooling is at least 1 to avoid nextInt(0)
                heat[i] = Math.max(0, heat[i] - rand.nextInt(cooling));
            }
            
            // Heat from each cell drifts up and diffuses
            for (int i = LedCount - 1; i >= 2; i--) {
                int sum = heat[i - 1] + heat[i - 2] + heat[i];
                heat[i] = sum / 3;
            }
            
            // Randomly ignite new sparks near the bottom
            if (rand.nextInt(255) < SPARKING) {
                int maxSparkPosition = Math.max(1, Math.min(7, LedCount - 1));
                int sparkPos = rand.nextInt(maxSparkPosition);
                int sparkHeat = rand.nextInt(95) + 160;
                heat[sparkPos] = Math.min(255, heat[sparkPos] + sparkHeat);
            }
            
            // Mark which LEDs have changed enough to warrant an update
            for (int i = 0; i < LedCount; i++) {
                // Only update if the value has changed by more than a threshold
                ledChanged[i] = Math.abs(heat[i] - prevHeat[i]) > 5;
            }
            
            // Update LEDs in small batches to reduce CAN utilization
            final int MAX_LEDS_PER_FRAME = 20; // Limit updates per frame
            int updatedCount = 0;
            
            for (int i = 0; i < LedCount && updatedCount < MAX_LEDS_PER_FRAME; i++) {
                if (ledChanged[i]) {
                    int heatValue = MathUtil.clamp(heat[i], 0, 255);
                    
                    // GREEN fire colors: green dominant, minimal red/blue
                    // More dramatic scaling to increase contrast
                    int r = (int)(heatValue * 0.2);  // Small red component
                    int g = heatValue;               // Full green
                    int b = (int)(heatValue * 0.1);  // Minimal blue
                    
                    // Display the fire right-side up (remove the inversion)
                    int displayPos = i;  // Use the position directly without inverting
                    
                    // Only update if within bounds
                    if (displayPos >= 0 && displayPos < LedCount) {
                        m_candleRight.setLEDs(r, g, b, 0, displayPos, 1);
                        m_candleLeft.setLEDs(r, g, b, 0, displayPos, 1);
                        updatedCount++;
                    }
                }
            }
            
        } catch (Exception e) {
            // Log the error and continue
            System.err.println("Error in runCustomFire: " + e.getMessage());
            e.printStackTrace(); // This will help diagnose the issue
            
            // Reset heat array in case it's corrupted
            for (int i = 0; i < LedCount; i++) {
                heat[i] = 0;
            }
        }
    }

    private void runCoralPulse() {
        try {
            // Create a pulsing effect between bright and dim purple
            float brightness = (float)(Math.sin(pulseCounter * 0.1) * 0.5 + 0.5); // Oscillate between 0-1
            
            // Purple color with varying brightness
            int r = (int)(200 * brightness);
            int g = 0;
            int b = (int)(255 * brightness);
            
            // Update all LEDs at once, but in batches to reduce CAN traffic
            final int batchSize = 20;
            final int totalBatches = (LedCount + batchSize - 1) / batchSize; // Ceiling division
            int currentBatch = (frameCounter / FRAME_SKIP) % totalBatches;
            int startIndex = currentBatch * batchSize;
            int count = Math.min(batchSize, LedCount - startIndex);
            
            if (startIndex < LedCount) {
                m_candleRight.setLEDs(r, g, b, 0, startIndex, count);
                m_candleLeft.setLEDs(r, g, b, 0, startIndex, count);
            }
            
        } catch (Exception e) {
            System.err.println("Error in runCoralPulse: " + e.getMessage());
        }
    }

    private void runAlgaePulse() {
        try {
            // Create a pulsing effect between bright and dim green
            float brightness = (float)(Math.sin(pulseCounter * 0.1) * 0.5 + 0.5); // Oscillate between 0-1
            
            // Bright green color with varying brightness
            int r = 0;
            int g = (int)(255 * brightness);
            int b = 0;
            
            // Update all LEDs at once, but in batches to reduce CAN traffic
            final int batchSize = 20;
            final int totalBatches = (LedCount + batchSize - 1) / batchSize; // Ceiling division
            int currentBatch = (frameCounter / FRAME_SKIP) % totalBatches;
            int startIndex = currentBatch * batchSize;
            int count = Math.min(batchSize, LedCount - startIndex);
            
            if (startIndex < LedCount) {
                m_candleRight.setLEDs(r, g, b, 0, startIndex, count);
                m_candleLeft.setLEDs(r, g, b, 0, startIndex, count);
            }
            
        } catch (Exception e) {
            System.err.println("Error in runAlgaePulse: " + e.getMessage());
        }
    }

    public Command setLEDAnimation(AnimationTypes animationType) {
        return Commands.runOnce(() -> setAnimation(animationType), this);
    }

    public Command turnOffLEDs() {
        return Commands.runOnce(() -> m_candleLeft.setLEDs(0, 0, 0, 0, 0, LedCount)).andThen(()->m_candleRight.setLEDs(0,0,0,0,0,LedCount));
    }
}
