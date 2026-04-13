package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedList;
import java.util.Queue;

public class fuelPerSecond {

    private boolean lastEnabled = false;

    private final Queue<Double> timestamps = new LinkedList<>();

    private double currentRate = 0;

    
    public void update(boolean shooting, boolean detected) {
        double now = Timer.getFPGATimestamp();

        
        if (shooting && !lastEnabled) {
            timestamps.clear();
            currentRate = 0;
        }

        if (shooting) {

            if (detected) {
                timestamps.add(now);
            }

            // Remove timestamps older than 1 second
            while (!timestamps.isEmpty() && now - timestamps.peek() > 1.0) {
                timestamps.poll();
            }

            // Update rate (gamepieces per second over last 1s window)
            currentRate = timestamps.size();
        }

        

        lastEnabled = shooting;
    }

    public double getRate() {
        return currentRate;
    }
}

