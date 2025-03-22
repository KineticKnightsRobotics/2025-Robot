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
        ColorFlow, Fire, Larson, Rainbow, RgbFade, SingleFade, Strobe, Twinkle, TwinkleOff,
        GamepieceAquired, Idle, ReadytoScore, 
        AutoDefault, AutoGamePiece, 
        EndgameYellow, EndgameOrange, EndgameRed,
        EndgameGamePiece,  // New animation type for game piece in endgame
        EndgameYellowWithGamepiece, EndgameOrangeWithGamepiece, EndgameRedWithGamepiece
    }

    private Animation m_currentAnimation;
    private AnimationTypes m_currentAnimationType;

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


        setAnimation(AnimationTypes.SingleFade);
    }

    @Override
    public void periodic() {
        
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
            case Idle:
                m_currentAnimation = new SingleFadeAnimation(50, 200, 2, 0, 0.5, LedCount);
                break;
            case GamepieceAquired:
                m_currentAnimation = new StrobeAnimation(50, 200, 2, 0, 98.0 / 256.0, LedCount);
                break;
            case ReadytoScore:
                m_currentAnimation = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
                
            // Autonomous animations
            case AutoDefault:
                m_currentAnimation = new FireAnimation(0.5, 0.8, LedCount-40, 0.9, 0.25);
                break;
            case AutoGamePiece:
                m_currentAnimation = new FireAnimation(1.0, 1.0, LedCount, 0.9, 0.25);
                break;
                
            // Endgame animations - single fade for no gamepiece
            case EndgameYellow:
                // Yellow fade animation
                m_currentAnimation = new SingleFadeAnimation(255, 255, 0, 0, 0.5, LedCount);
                break;
            case EndgameOrange:
                // Orange fade animation
                m_currentAnimation = new SingleFadeAnimation(255, 165, 0, 0, 0.5, LedCount);
                break;
            case EndgameRed:
                // Red fade animation
                m_currentAnimation = new SingleFadeAnimation(255, 0, 0, 0, 0.5, LedCount);
                break;
            case EndgameGamePiece:
                m_currentAnimation = new StrobeAnimation(0, 255, 0, 0, 0.8, LedCount);
                break;
                
            // Endgame with gamepiece - Fixed constructors with valid parameters
            case EndgameYellowWithGamepiece:
                m_currentAnimation = new StrobeAnimation(255, 255, 0, 0, 0.9, LedCount);
                ((StrobeAnimation)m_currentAnimation).setSpeed(0.4); // Make it blink faster
                break;
            case EndgameOrangeWithGamepiece:
                m_currentAnimation = new StrobeAnimation(255, 165, 0, 0, 0.9, LedCount);
                ((StrobeAnimation)m_currentAnimation).setSpeed(0.4); // Make it blink faster
                break;
            case EndgameRedWithGamepiece:
                m_currentAnimation = new StrobeAnimation(255, 0, 0, 0, 0.9, LedCount);
                ((StrobeAnimation)m_currentAnimation).setSpeed(0.4); // Make it blink faster
                break;

            // Standard animations
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
            default:
                m_currentAnimation = null;
                break;
        }
    }

    public Command setLEDAnimation(AnimationTypes animationType) {
        return Commands.runOnce(() -> setAnimation(animationType), this);
    }

    public Command turnOffLEDs() {
        return Commands.runOnce(() -> {m_candleLeft.setLEDs(0, 0, 0, 0, 0, LedCount);m_candleRight.setLEDs(0,0,0,0,0,LedCount);});
    }
}
