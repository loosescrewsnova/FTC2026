package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


//L2 left trigger is the button
public class pollenpusherbutton {


    private final MecanumDrive drive;


    // ----------------------------
    // TUNING
    // ----------------------------


    private static final double FIRST_SWEEP_DEGREES = 45.0;
    private static final double SECOND_SWEEP_DEGREES = 90.0;


    // true  -> the BACK end sweeps left first, then right (what was asked for).
    // false -> mirrored: the back end sweeps right first, then left.
    // If the back swings the wrong way on the real robot, flip this.
    private static final boolean BACK_SWEEPS_LEFT_FIRST = true;


    // PS4 L2 = gamepad.left_trigger, which reads 0.0 to 1.0.
    // Counts as "pressed" once it goes past this value.
    private static final double TRIGGER_THRESHOLD = 0.5;


    private static final double KP = 1.2;                    // power per radian of error
    private static final double MAX_TURN_POWER = 0.5;
    private static final double MIN_TURN_POWER = 0.12;       // keeps it from stalling near the target
    private static final double TOLERANCE_DEGREES = 2.0;     // "close enough" window
    private static final double TURN_TIMEOUT_SECONDS = 2.0;  // safety: give up on one turn after this


    // ----------------------------
    // STATE
    // ----------------------------


    private enum State { IDLE, SWEEP_FIRST, SWEEP_SECOND }


    private State state = State.IDLE;
    private boolean wasPressed = false;


    private double targetHeading = 0.0;   // what we're turning toward right now
    private double secondTarget = 0.0;    // where the second sweep ends


    private final ElapsedTime turnTimer = new ElapsedTime();


    public pollenpusherbutton(HardwareMap drive) {
        this.drive = drive;
    }




    public void update(double leftTrigger, double yawRadians) {




        boolean pressed = leftTrigger > TRIGGER_THRESHOLD;
        boolean justPressed = pressed && !wasPressed;
        wasPressed = pressed;
        if (!pressed && state != State.IDLE) {
            cancel();
            return;
        }


        double heading = drive.getHeadingRadians();


        switch (state) {


            case IDLE:
                if (justPressed) {
                    double firstSign = BACK_SWEEPS_LEFT_FIRST ? -1.0 : 1.0;


                    targetHeading = normalizeAngle(
                            heading + firstSign * Math.toRadians(FIRST_SWEEP_DEGREES));


                    secondTarget = normalizeAngle(
                            targetHeading - firstSign * Math.toRadians(SECOND_SWEEP_DEGREES));


                    turnTimer.reset();
                    state = State.SWEEP_FIRST;
                }
                break;


            case SWEEP_FIRST:
                if (turnTowards(targetHeading, heading)) {
                    targetHeading = secondTarget;
                    turnTimer.reset();
                    state = State.SWEEP_SECOND;
                }
                break;


            case SWEEP_SECOND:
                if (turnTowards(targetHeading, heading)) {
                    state = State.IDLE;
                }
                break;
        }
    }




    private boolean turnTowards(double target, double heading) {


        double error = normalizeAngle(target - heading);


        if (Math.abs(error) < Math.toRadians(TOLERANCE_DEGREES)
                || turnTimer.seconds() > TURN_TIMEOUT_SECONDS) {
            drive.mecanumDrive(0, 0, 0);
            return true;
        }
        double power = Math.abs(error) * KP;
        power = Math.max(MIN_TURN_POWER, Math.min(MAX_TURN_POWER, power));




        double rx = -Math.signum(error) * power;


        drive.mecanumDrive(0, 0, rx);
        return false;
    }


    public boolean isBusy() {
        return state != State.IDLE;
    }


    public void cancel() {
        state = State.IDLE;
        drive.mecanumDrive(0, 0, 0);
    }


    public String getStateName() {
        return state.name();
    }
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}

