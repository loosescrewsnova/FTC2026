package org.firstinspires.ftc.teamcode.Mains;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Runtime.getRuntime;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.IMUOrthogonalNew;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.NoOdometryMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.limelightAutoalign;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightLocalizer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Runtime.getRuntime;






    @TeleOp(name = "Limelight TeleOp")
    public class AutoAlignMain extends OpMode {

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
            limelightAutoalign aligner;
            Limelight3A limelight;

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(8);
            limelight.start();
            aligner = new limelightAutoalign(limelight, 34);   // starting tag ID

            if (gamepad1.right_bumper) {   // R1
                aligner.update();
                rx = aligner.turn;
                y += aligner.forward;
            }

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
            IMUOrthogonalNew imuOrthogonal = null;
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

            intake.intake(gamepad1);
            feeder.feeder(gamepad1);
            shooter.shooter(gamepad1);

            // =================================
            // 4. ENDGAME RUMBLE
            // =================================

            if (getRuntime() >= endGameStart && !isEndGame) {

                gamepad1.rumbleBlips(3);


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


