package org.firstinspires.ftc.teamcode.Mains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.IMUOrthogonalNew;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.PIDController;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;


@TeleOp(name = "Joystick Operations")
public class Main extends OpMode {

    private MecanumDrive mecanumDrive;
    private Intake intake;
    private Feeder feeder;
    private IMUOrthogonalNew imuOrthogonal;
    private LimelightLocalizer limelightLocalizer;
    private PIDController turretPID;

    private Shooter shooter;


    private double endGameStart;
    private boolean isEndGame;

    // ----------------------------
    // TURRET MODE TUNING
    // ----------------------------
    // Tune kP first (start small, increase until tracking is snappy
    // without oscillating back and forth), then add a little kD to damp
    // any overshoot. kI is rarely needed for this kind of angular
    // tracking - leave it at 0 unless you see a persistent steady-state
    // error that P and D alone won't clear.
    private static final double TURRET_kP = 0.03;
    private static final double TURRET_kI = 0.0;
    private static final double TURRET_kD = 0.002;

    // Turret rotation power is capped well below full power - this is a
    // fine tracking correction layered under normal driving, not a full
    // drive command.
    private static final double TURRET_MAX_POWER = 0.5;

    @Override
    public void init() {

        mecanumDrive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        feeder = new Feeder(hardwareMap);
        imuOrthogonal = new IMUOrthogonalNew(hardwareMap);
        limelightLocalizer = new LimelightLocalizer(hardwareMap);
        turretPID = new PIDController(TURRET_kP, TURRET_kI, TURRET_kD);
        shooter = new Shooter(hardwareMap);


        isEndGame = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {

        limelightLocalizer.start();

        endGameStart = getRuntime() + 90;
    }

    @Override
    public void loop() {

        // Hold left trigger = intake, hold right trigger = feeder
        intake.intake(gamepad2);
        feeder.feeder(gamepad2);
        shooter.shooter(gamepad2);


        double y = gamepad1.right_stick_y;
        double x = -gamepad1.right_stick_x;
        double rx;

        // Hold right bumper to enable turret mode: the robot auto-rotates
        // to keep the AprilTag centered in the Limelight's view while you
        // still freely control translation (y, x) with the sticks. Let go
        // of the bumper at any time to take back manual rotation control.
        boolean turretModeRequested = gamepad2.right_bumper;

        // 1. Limelight reads AprilTags and updates the IMU's heading offset.
        limelightLocalizer.update(imuOrthogonal.getYawDegrees());

        boolean hasTarget = limelightLocalizer.hasValidTarget();
        boolean turretActive = turretModeRequested && hasTarget;

        if (turretActive) {

            // tx: degrees between the crosshair and the tag. 0 = centered,
            // positive = tag is to the right. Driving this to 0 with the
            // PID is what keeps the tag in view while you strafe.
            double txError = limelightLocalizer.getTx();

            double turretPower = turretPID.calculate(txError);

            turretPower = Math.max(
                    -TURRET_MAX_POWER,
                    Math.min(TURRET_MAX_POWER, turretPower)
            );

            rx = turretPower;

            // If the robot turns AWAY from the tag instead of toward it
            // once you test this, the sign is flipped for this
            // drivetrain's rotation convention - swap in this line:
            // rx = -turretPower;

        } else {

            rx = -gamepad1.left_stick_x;

            // Reset whenever turret mode is off or the tag isn't visible,
            // so stale integral/derivative state doesn't cause a jerky
            // snap the next time turret mode engages.
            turretPID.reset();
        }

        mecanumDrive.mecanumDrive(y, x, rx);

        if (hasTarget) {

            imuOrthogonal.correctYaw(
                    limelightLocalizer.getFieldHeadingRadians()
            );

            mecanumDrive.applyVisionCorrection(
                    limelightLocalizer.getFieldXInches(),
                    limelightLocalizer.getFieldYInches()
            );
        }

        // 2. The (possibly corrected) IMU heading updates the encoders'
        //    position calculation.
        mecanumDrive.updateOdometry(imuOrthogonal.getYawRadians());

        if (gamepad2.y) {
            imuOrthogonal.resetYaw();
        }

        if (getRuntime() >= endGameStart && !isEndGame) {

            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
            isEndGame = true;
        }

        // ----------------------------
        // TELEMETRY (kept short on purpose)
        // ----------------------------
        telemetry.addData(
                "Pose",
                "X %.1f in | Y %.1f in | H %.0f deg",
                mecanumDrive.getX(),
                mecanumDrive.getY(),
                mecanumDrive.getHeadingDegrees()
        );

        telemetry.addData(
                "Intake / Feeder / Shooter",
                "%.2f / %.2f / %.2f",
                intake.getIntakePower(),
                feeder.getFeederPower(),
                shooter.getShooterSpeed()
        );

        telemetry.addData(
                "Vision / Turret",
                "%s / %s",
                hasTarget ? "TAG" : "none",
                turretActive ? "ON" : "off"
        );

        telemetry.addData("Endgame", isEndGame);

        telemetry.update();
    }
}