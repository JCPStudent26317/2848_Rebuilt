package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Timer;

public class RangerHelpers {
    
    public static void setupTalonFX(TalonFX talon, TalonFXConfiguration config) {
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = talon.getConfigurator().apply(config);
            if (status.isOK()) break;
            Timer.delay(0.01); // 10 ms delay between retries
        }

        if (!status.isOK()) {
            System.out.println(
                "TalonFX " + talon.getDeviceID() +
                " could not apply configs, error code: " + status
            );
        }
    }
}
