package org.firstinspires.ftc.teamcode.Mains;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Mecanum Drive Only", group = "TeleOp")
public class MecanumDriveMain extends OpMode {

    // Drivetrain motors
    private DcMotor rightFrontMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftBackMotor;

    // Maximum drivetrain power
    private static final double MAX_SPEED = 0.8;

    @Override
    public void init() {

        // Initialize motors using the Robot Configuration names
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");

        // Reverse the right-side motors
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use the drivetrain without encoders
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero-power behavior
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop all motors initially
        stopMotors();

        telemetry.addLine("Mecanum Drive Initialized");
        telemetry.addLine("Ready to drive!");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Gamepad 1 controls
        double y = -gamepad1.left_stick_y;  // Forward / backward
        double x = gamepad1.left_stick_x;   // Strafe left / right
        double rx = gamepad1.right_stick_x; // Rotate left / right

        // Mecanum wheel power calculations
        double leftFrontPower = y + x + rx;
        double rightFrontPower = y - x - rx;
        double leftBackPower = y - x + rx;
        double rightBackPower = y + x - rx;

        // Normalize powers so none exceed 1.0
        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rx),
                1.0
        );

        leftFrontPower /= denominator;
        rightFrontPower /= denominator;
        leftBackPower /= denominator;
        rightBackPower /= denominator;

        // Apply maximum speed limit
        leftFrontPower *= MAX_SPEED;
        rightFrontPower *= MAX_SPEED;
        leftBackPower *= MAX_SPEED;
        rightBackPower *= MAX_SPEED;

        // Set motor powers
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);

        // Telemetry
        telemetry.addData("Forward/Backward", "%.2f", y);
        telemetry.addData("Strafe", "%.2f", x);
        telemetry.addData("Turn", "%.2f", rx);

        telemetry.addData("Left Front Power", "%.2f", leftFrontPower);
        telemetry.addData("Right Front Power", "%.2f", rightFrontPower);
        telemetry.addData("Left Back Power", "%.2f", leftBackPower);
        telemetry.addData("Right Back Power", "%.2f", rightBackPower);

        telemetry.update();
    }

    private void stopMotors() {
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftBackMotor.setPower(0);
    }

    @Override
    public void stop() {
        stopMotors();
    }
}