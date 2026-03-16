package frc.robot.subsystems;

import static frc.robot.Constants.LightsConstants.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RangerHelpers;
import frc.robot.RangerHelpers.ActiveAlliance;;

public class Lights extends SubsystemBase {
    
    private final CANdle candle = new CANdle(kCANdleID);
    private final CANdleConfiguration candleconfig = new CANdleConfiguration();

    private enum AnimationType {
        SOLID,
        STROBE_SLOW,
        STROBE_FAST
    }

    private ControlRequest candleRequest = kRainbow;
    private ActiveAlliance previousActiveAlliance = ActiveAlliance.NONE;
    private AnimationType previousAnimationType = AnimationType.SOLID;

    public void clearAnimations() {
        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }
    }

    public Lights() {
        candleconfig.LED.StripType = StripTypeValue.GRB;
        candleconfig.LED.BrightnessScalar = 1;
        candleconfig.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        
        candle.getConfigurator().apply(candleconfig);

        clearAnimations();
    }

    @Override
    public void periodic() {
        candle.setControl(candleRequest);

        SmartDashboard.putData(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Raw Game Time", () -> DriverStation.getMatchTime(), null);
        builder.addIntegerProperty("Remaining Shift Time", () -> RangerHelpers.getRemainingShiftTime(), null);
        builder.addStringProperty("Active Alliance", () -> {
            switch(RangerHelpers.getActiveAlliance()) {
                case RED: return "Red";
                case BLUE: return "Blue";
                case BOTH: return "Both";
                case NONE: return "None";
                default: return "???";
            }
        }, null);
    }

    public Command runPattern(ControlRequest request) {
        return Commands.runOnce(() -> {
            clearAnimations();
            candleRequest = request;
        }, this);
    }

    public Command signalActiveAlliance() {
        return Commands.run(() -> {
            ActiveAlliance nextActiveAlliance = RangerHelpers.getActiveAlliance();
            AnimationType nextAnimationType;

            if (RangerHelpers.getRemainingShiftTime() == -1) {
                nextAnimationType = AnimationType.SOLID;
            } else if (RangerHelpers.getRemainingShiftTime() <= 2) {
                nextAnimationType = AnimationType.STROBE_FAST;
            } else if (RangerHelpers.getRemainingShiftTime() <= 4) {
                nextAnimationType = AnimationType.STROBE_SLOW;
            } else {
                nextAnimationType = AnimationType.SOLID;
            }
            
            if (nextActiveAlliance != previousActiveAlliance || nextAnimationType != previousAnimationType) {
                ControlRequest request;
                RGBWColor color;

                switch(nextActiveAlliance) {
                    case RED: 
                        color = kRed;
                        break;
                    case BLUE:
                        color = kBlue;
                        break;
                    case BOTH: 
                        color = kGreen;
                        break;
                    case NONE:
                        color = kBlack;
                        break;
                    default:
                        color = kWhite;
                        break;
                }

                switch(nextAnimationType) { 
                    case STROBE_FAST:
                        request = new StrobeAnimation(kStartIndex, kEndIndex).withFrameRate(8).withColor(color).withSlot(0);
                        break;
                    case STROBE_SLOW:
                        request = new StrobeAnimation(kStartIndex, kEndIndex).withFrameRate(4).withColor(color).withSlot(0);
                        break;
                    case SOLID:
                        request = new SolidColor(kStartIndex, kEndIndex).withColor(color);
                        break;
                    default:
                        request = new TwinkleAnimation(kStartIndex, kEndIndex).withColor(color).withSlot(0);
                        break;
                }
                clearAnimations();
                candleRequest = request;

                previousActiveAlliance = nextActiveAlliance;
                previousAnimationType = nextAnimationType;
            }
        }, this);
    }

}
