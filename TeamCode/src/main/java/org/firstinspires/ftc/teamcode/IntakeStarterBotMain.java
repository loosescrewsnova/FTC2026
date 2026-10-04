package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IMUOrthogonalNew;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.PIDController;

@TeleOp(name = "No Shooter Code")
public class IntakeStarterBotMain extends OpMode {

    private MecanumDrive mecanumDrive;
    private Intake intake;
    private IMUOrthogonalNew imuOrthogonal;

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
        imuOrthogonal = new IMUOrthogonalNew(hardwareMap);

        isEndGame = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {

        endGameStart = getRuntime() + 90;
    }

    @Override
    public void loop() {

        intake.intake(gamepad1);

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;;

        mecanumDrive.mecanumDrive(y, x, rx);

        // Hold right bumper to enable turret mode: the robot auto-rotates
        // to keep the AprilTag centered in the Limelight's view while you
        // still freely control translation (y, x) with the sticks. Let go
        // of the bumper at any time to take back manual rotation control.
        boolean turretModeRequested = gamepad1.right_bumper;

        // 2. The (possibly corrected) IMU heading updates the encoders'
        //    position calculation.
        mecanumDrive.updateOdometry(imuOrthogonal.getYawRadians());

        if (gamepad1.y) {
            imuOrthogonal.resetYaw();
        }

        if (getRuntime() >= endGameStart && !isEndGame) {

            gamepad1.rumbleBlips(3);
            isEndGame = true;
        }

        telemetry.addData(
                "Front Left Power",
                mecanumDrive.getLeftFrontPower()
        );

        telemetry.addData(
                "Front Right Power",
                mecanumDrive.getRightFrontPower()
        );

        telemetry.addData(
                "Back Left Power",
                mecanumDrive.getLeftBackPower()
        );

        telemetry.addData(
                "Back Right Power",
                mecanumDrive.getRightBackPower()
        );

        telemetry.addData(
                "Intake Power",
                intake.getIntakePower()
        );

        telemetry.addData(
                "Hub Orientation",
                "Logo=%s USB=%s",
                imuOrthogonal.getLogoDirection(),
                imuOrthogonal.getUsbDirection()
        );

        telemetry.addData(
                "Yaw (Z)",
                "%.2f Deg.",
                imuOrthogonal.getYawDegrees()
        );

        telemetry.addData(
                "Pitch (X)",
                "%.2f Deg.",
                imuOrthogonal.getPitchDegrees()
        );

        telemetry.addData(
                "Roll (Y)",
                "%.2f Deg.",
                imuOrthogonal.getRollDegrees()
        );

        telemetry.addData(
                "Yaw Velocity",
                "%.2f Deg/Sec",
                imuOrthogonal.getYawVelocity()
        );

        telemetry.addData(
                "Pitch Velocity",
                "%.2f Deg/Sec",
                imuOrthogonal.getPitchVelocity()
        );

        telemetry.addData(
                "Roll Velocity",
                "%.2f Deg/Sec",
                imuOrthogonal.getRollVelocity()
        );

        telemetry.addData(
                "X",
                "%.2f in",
                mecanumDrive.getX()
        );

        telemetry.addData(
                "Y",
                "%.2f in",
                mecanumDrive.getY()
        );

        telemetry.addData(
                "Heading",
                "%.2f Deg",
                mecanumDrive.getHeadingDegrees()
        );

        telemetry.addData(
                "Is it Endgame",
                isEndGame
        );

        telemetry.update();
    }
}