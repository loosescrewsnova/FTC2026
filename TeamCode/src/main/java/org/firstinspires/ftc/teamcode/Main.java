package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IMUOrthogonalNew;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp(name = "Joystick Operations")
public class Main extends OpMode {

    private MecanumDrive mecanumDrive;
    private Intake intake;
    private Shooter shooter;
    private IMUOrthogonalNew imuOrthogonal;

    private double endGameStart;
    private boolean isEndGame;

    @Override
    public void init() {

        mecanumDrive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
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

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        mecanumDrive.mecanumDrive(y, x, rx);

        // Update odometry using the IMU
        mecanumDrive.updateOdometry(imuOrthogonal.getIMU());

        intake.intake(gamepad1);

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