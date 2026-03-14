package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

/**
 * Utility helper methods for configuring CTRE TalonFX motor controllers
 * and other CAN devices with built-in retry logic to handle transient CAN errors,
 * plus determining what alliance is active at any time.
 */
public class RangerHelpers {
  
  /**
   * Applies a {@link TalonFXConfiguration} to a TalonFX motor controller,
   * retrying up to 5 times if the configuration fails to apply.
   *
   * <p>This is useful for mitigating occasional CAN bus timing issues
   * during robot startup or brownout recovery.</p>
   *
   * @param talon the TalonFX motor controller to configure
   * @param config the configuration object to apply to the motor controller
   */
  public static void setupTalonFx(TalonFX talon, TalonFXConfiguration config) {
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; ++i) {
      status = talon.getConfigurator().apply(config);
      if (status.isOK()) {
        break;
      }
      Timer.delay(0.01); // 10 ms delay between retries
    }

    if (!status.isOK()) {
      System.out.println(
          "TalonFX " + talon.getDeviceID() +
          " could not apply configs, error code: " + status
      );
    }
  }

  /**
   * Executes a configuration apply operation with retry logic.
   *
   * <p>The supplied function is called up to 5 times or until it returns
   * a successful {@link StatusCode}. If all attempts fail, an assertion
   * is triggered.</p>
   *
   * <p>This method is intended for generic configuration calls where the
   * apply logic is provided via a lambda or method reference.</p>
   *
   * @param toApply a {@link Supplier} that performs a configuration apply
   *                and returns the resulting {@link StatusCode}
   */
  public static void retryConfigApply(Supplier<StatusCode> toApply) {
    StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
    int triesLeftOver = 5;
    do {
      finalCode = toApply.get();
    } while (!finalCode.isOK() && --triesLeftOver > 0);
      assert (finalCode.isOK());
  }

  /** 
   * Returns true when our alliance's hub is active.
   * Copied from WPILib docs: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
   *  
   * @return If our alliance's hub is active
  */
  public static boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  public static enum ActiveAlliance {
    RED,
    BLUE,
    BOTH,
    NONE
  }

  /** 
   * Returns the active alliance.
   * Copied and modified from WPILib docs: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
   * 
   * @return Active alliance state
  */
  public static ActiveAlliance getActiveAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return ActiveAlliance.NONE;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return ActiveAlliance.BOTH;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return ActiveAlliance.NONE;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return ActiveAlliance.BOTH;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return ActiveAlliance.BOTH;
      }
    }

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return ActiveAlliance.BOTH;
    } else if (matchTime > 105) {
      // Shift 1
      return redInactiveFirst ? ActiveAlliance.BLUE : ActiveAlliance.RED;
    } else if (matchTime > 80) {
      // Shift 2
      return redInactiveFirst ? ActiveAlliance.RED : ActiveAlliance.BLUE;
    } else if (matchTime > 55) {
      // Shift 3
      return redInactiveFirst ? ActiveAlliance.BLUE : ActiveAlliance.RED;
    } else if (matchTime > 30) {
      // Shift 4
      return redInactiveFirst ? ActiveAlliance.RED : ActiveAlliance.BLUE;
    } else {
      // End game, hub always active.
      return ActiveAlliance.BOTH;
    }
  }

  /** 
   * Returns the number of seconds remaining in the shift, as an integer, during DS practice matches or on the real field.
   * Returns -1 if the robot is not enabled or if there is no hub.
   * Will behave differently during DS teleop or auto modes.
   * 
   * @return Number of seconds reamining in the shift
  */  
  public static int getRemainingShiftTime() {
    // getMatchTime() returns an integer on the real field and a floating point number outside of the real field
    // This is to standardize it.
    int matchTime = (int) Math.floor(DriverStation.getMatchTime() + 0.0001);
    
    if (DriverStation.getAlliance().isEmpty()) {
      return -1;
    }
    // Autonomous
    if (DriverStation.isAutonomousEnabled()) {
      return matchTime;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return -1;
    }
    
    // If there is no game data we will assume it is the start of the transition phase.
    if (DriverStation.getGameSpecificMessage().isEmpty()) {
      return 10;
    }

    // Transition shift
    if (matchTime > 130) {
      return matchTime - 10;
    }

    // Shifts 1-4
    if (matchTime > 30) {
      int mod = (matchTime - 30) % 25;
      return mod == 0 ? 25 : mod;
    }

    // Endgame
    return matchTime;
  }  

}
