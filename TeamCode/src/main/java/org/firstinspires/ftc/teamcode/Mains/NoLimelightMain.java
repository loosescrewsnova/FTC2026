package org.firstinspires.ftc.teamcode.Mains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.shooter.Shooter;

@TeleOp(name = "Joystick Operations")
public class NoLimelightMain extends OpMode {

    // Subsystems
    private MecanumDrive mecanumDrive;
    private Intake intake;
    private Feeder feeder;
    private Shooter shooter;

    // Endgame
    private double endGameStart;
    private boolean isEndGame;

    // Driver controls
    private double y;
    private double x;
    private double rx;

    @Override
    public void init() {

        // Initialize all subsystems
        mecanumDrive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);

        NoOdometryMecanum mecanum = new NoOdometryMecanum(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Feeder feeder = new Feeder(hardwareMap);

        isEndGame = false;

        telemetry.addLine("Robot Initialized");
        telemetry.addData("Drive", "Ready");
        telemetry.addData("Intake", "Ready");
        telemetry.addData("Feeder", "Ready");
        telemetry.addData("Shooter", "Ready");
        telemetry.addData("IMU", "Initialized");

        telemetry.update();
    }

    @Override
    public void start() {

        // Endgame begins 90 seconds after TeleOp starts
        endGameStart = getRuntime() + 90.0;
        isEndGame = false;
    }

    @Override
    public void loop() {

        // =================================
        // 1. DRIVETRAIN CONTROLS
        // =================================

        // Forward/backward
        y = -gamepad1.right_stick_y;

        // Strafe left/right
        x = -gamepad1.right_stick_x;

        // Rotation
        rx = -gamepad1.left_stick_x;

        // Send joystick inputs to the drivetrain
        mecanumDrive.mecanumDrive(y, x, rx);

        // =================================
        // 2. IMU AND ODOMETRY
        // =================================

        // Reset heading when Y is pressed
        if (gamepad1.y) {
            imuOrthogonal.resetYaw();
        }

        // Update odometry using the latest IMU heading
        mecanumDrive.updateOdometry(
                imuOrthogonal.getYawRadians()
        );

        // =================================
        // 3. INTAKE, FEEDER, AND SHOOTER
        // =================================

        intake.intake(gamepad2);
        feeder.feeder(gamepad2);
        shooter.shooter(gamepad2);

        // =================================
        // 4. ENDGAME RUMBLE
        // =================================

        if (getRuntime() >= endGameStart && !isEndGame) {

            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);

            isEndGame = true;
        }

        // =================================
        // 5. DRIVETRAIN TELEMETRY
        // =================================

        telemetry.addLine("===== DRIVETRAIN =====");

        telemetry.addData(
                "Joystick Y / X / Rotation",
                "%.2f / %.2f / %.2f",
                y, x, rx
        );

        telemetry.addData(
                "Robot X (in)",
                "%.2f",
                mecanumDrive.getX()
        );

        telemetry.addData(
                "Robot Y (in)",
                "%.2f",
                mecanumDrive.getY()
        );

        telemetry.addData(
                "Heading (degrees)",
                "%.1f",
                mecanumDrive.getHeadingDegrees()
        );

        // =================================
        // 6. MECHANISM TELEMETRY
        // =================================

        telemetry.addLine("===== MECHANISMS =====");

        telemetry.addData(
                "Intake Power",
                "%.2f",
                intake.getIntakePower()
        );

        telemetry.addData(
                "Feeder Power",
                "%.2f",
                feeder.getFeederPower()
        );

        telemetry.addData(
                "Shooter Speed",
                "%.2f",
                shooter.getShooterSpeed()
        );

        // =================================
        // 7. MATCH TELEMETRY
        // =================================

        telemetry.addLine("===== MATCH STATUS =====");

        telemetry.addData(
                "Match Time (seconds)",
                "%.1f",
                getRuntime()
        );

        telemetry.addData(
                "Endgame",
                isEndGame ? "ACTIVE" : "Not yet"
        );

        telemetry.addData(
                "Time Until Endgame",
                "%.1f seconds",
                Math.max(0, endGameStart - getRuntime())
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        // Stop drivetrain motors when the OpMode ends
        mecanumDrive.mecanumDrive(0, 0, 0);

        telemetry.addLine("OpMode Stopped");
        telemetry.update();
    }
}
