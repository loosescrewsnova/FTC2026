package org.firstinspires.ftc.teamcode.Mains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.IMUOrthogonalNew;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.PIDController;
import org.firstinspires.ftc.teamcode.Subsystems.shooter.Shooter;


@TeleOp(name = "Joystick Operations")
public class NoLimelightMain extends OpMode {

    private MecanumDrive mecanumDrive;
    private Intake intake;
    private Feeder feeder;
    private IMUOrthogonalNew imuOrthogonal;
    private Shooter shooter;


    private double endGameStart;
    private boolean isEndGame;

    @Override
    public void init() {

        mecanumDrive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        feeder = new Feeder(hardwareMap);
        imuOrthogonal = new IMUOrthogonalNew(hardwareMap);
        shooter = new Shooter(hardwareMap);


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

        // Hold left trigger = intake, hold right trigger = feeder
        intake.intake(gamepad1);
        feeder.feeder(gamepad1);
        shooter.shooter(gamepad1);


        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx;

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

        telemetry.addData("Endgame", isEndGame);

        telemetry.update();
    }
}