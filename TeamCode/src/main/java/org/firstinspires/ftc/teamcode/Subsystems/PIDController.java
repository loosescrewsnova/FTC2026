package org.firstinspires.ftc.teamcode.Subsystems;

/*
 * Simple, reusable PID controller with two safety details baked in:
 *
 * 1. Integral clamping (setIntegralLimit) - stops the integral term from
 *    winding up to a huge value if error sits nonzero for a while, which
 *    would otherwise cause a big overshoot once the error finally clears.
 *
 * 2. First-sample-after-reset handling - the very first calculate() call
 *    after construction or reset() only applies the P term. Without this,
 *    dt would be tiny and lastError would be stale/zero, causing a huge
 *    derivative "kick" (a visible twitch) the instant a controller like
 *    the turret-aim PID turns on.
 */
public class PIDController {

    private final double kP;
    private final double kI;
    private final double kD;

    private double integralSum = 0;
    private double lastError = 0;
    private long lastTimeNanos;
    private double integralLimit = Double.MAX_VALUE;
    private boolean firstSampleSinceReset = true;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // Caps how far the integral sum can wind up in either direction.
    public void setIntegralLimit(double limit) {
        this.integralLimit = Math.abs(limit);
    }

    public double calculate(double error) {

        long now = System.nanoTime();

        if (firstSampleSinceReset) {
            // No elapsed time yet, so there's nothing to integrate or
            // differentiate against - seed the state and return P only.
            lastTimeNanos = now;
            lastError = error;
            firstSampleSinceReset = false;
            return kP * error;
        }

        double dt = (now - lastTimeNanos) / 1e9;
        lastTimeNanos = now;

        if (dt <= 0) {
            dt = 1e-3; // guards against a zero/negative clock read
        }

        integralSum += error * dt;
        integralSum = Math.max(-integralLimit, Math.min(integralLimit, integralSum));

        double derivative = (error - lastError) / dt;
        lastError = error;

        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    // Call this whenever the controller is disabled/re-enabled (e.g. the
    // turret mode button is released, or the target is lost) so stale
    // integral/derivative state doesn't cause a jerky snap next time it
    // engages.
    public void reset() {
        integralSum = 0;
        lastError = 0;
        firstSampleSinceReset = true;
    }
}