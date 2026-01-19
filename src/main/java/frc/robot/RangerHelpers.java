package frc.robot;

import java.util.function.Supplier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Timer;

/**
 * Utility helper methods for configuring CTRE TalonFX motor controllers
 * and other CAN devices with built-in retry logic to handle transient CAN errors.
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
}
