package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.pedro.Constants;

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

    // PedroPathing tunes heading error in radians. This controller receives
    // the Limelight error converted to radians by Main, so the tuned gain is
    // directly applicable here. Pedro's Foresight tuning does not provide
    // turret-specific integral or derivative gains.
    private final double kP = Constants.HEADING_FEEDBACK_KP;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double integralSum = 0;
    private double lastError = 0;
    private long lastTimeNanos;
    private double integralLimit = Double.MAX_VALUE;
    private boolean firstSampleSinceReset = true;

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