package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedList;
import java.util.Queue;

public class fuelPerSecond {

    private boolean lastEnabled = false;

    private final Queue<Double> timestamps = new LinkedList<>();

    private double currentRate = 0.0;

    /**
     * Call this every robot loop (periodic)
     * Handles sensor state, edge detection, and rate calculation
     */
    public void update(boolean enabled, boolean sensorTriggered) {
        double now = Timer.getFPGATimestamp();

        // Detect rising edge of enable (false -> true)
        if (enabled && !lastEnabled) {
            timestamps.clear();
            currentRate = 0;
        }

        // Only process sensor while enabled
        if (enabled) {

            // Count gamepiece on sensor trigger (edge-triggered externally)
            if (sensorTriggered) {
                timestamps.add(now);
            }

            // Remove timestamps older than 1 second
            while (!timestamps.isEmpty() && now - timestamps.peek() > 1.0) {
                timestamps.poll();
            }

            // Update rate (gamepieces per second over last 1s window)
            currentRate = timestamps.size();
        }

        // If disabled → freeze value (do nothing)

        lastEnabled = enabled;
    }

    /**
     * Pure getter — safe to call anywhere
     */
    public double getRate() {
        return currentRate;
    }
}

