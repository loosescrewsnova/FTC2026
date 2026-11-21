package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.tuning.autotune.DisplayName;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Function;
import java.util.function.Supplier;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

public class Tests extends Procedure {
    enum Test {
        @DisplayName("Hold Test")
        HOLD,
        @DisplayName("Line Test")
        LINE,
        @DisplayName("Curve Test")
        CURVED,
        @DisplayName("Interpolation Test")
        INTERPOLATION_CURVED,
        @DisplayName("Localization Test")
        LOCALIZATION,
        @DisplayName("Odometry Test")
        ODOMETRY,
        @DisplayName("Driving Test")
        DRIVING,
        @DisplayName("Pose Test")
        POSE
    }
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    Function<HardwareMap, Localizer> localizerFunction;
    Supplier<Algorithm> algorithmSupplier;
    Function<HardwareMap, Follower> followerFunction;

    public Tests(Function<HardwareMap, Drivetrain> drivetrainFunction, Function<HardwareMap, Localizer> localizerFunction, Supplier<Algorithm> algorithmSupplier) {
        super("Tests", "A procedure for testing the Follower.");
        this.drivetrainFunction = drivetrainFunction;
        this.localizerFunction = localizerFunction;
        this.algorithmSupplier = algorithmSupplier;
    }

    @Override
    public void run() throws InterruptedException {
        boolean completed = false;
        boolean algorithm = algorithmSupplier != null;
        boolean localizer = localizerFunction != null;
        boolean drivetrain = drivetrainFunction != null;
        boolean followerConfigured = algorithm && localizer && drivetrain;

        if (followerConfigured)
            followerFunction = (hardwareMap) -> new Follower(localizerFunction.apply(hardwareMap), drivetrainFunction.apply(hardwareMap), algorithmSupplier.get());

        Inputs inputs = inputs("Select", "Select");
        Inputs.Field<Test> selectedTest = inputs.e("Test", Test.class).withDefault(Test.LINE);
        Inputs.Field<Double> distance = inputs.d("Distance").withDefault(48.0);

        awaitInputs(inputs);

        switch (selectedTest.get()) {
            case HOLD:
                if (!followerConfigured) {
                    abort("Hold Test requires a configured algorithm, localizer, and drivetrain.");
                    return;
                }
                completed = runOpMode(new TestsHold(followerFunction));
                break;
            case LINE:
                if (!followerConfigured) {
                    abort("Line Test requires a configured algorithm, localizer, and drivetrain.");
                    return;
                }
                if (!(distance.get() > 0.0) || !Double.isFinite(distance.get())) {
                    abort("Test distance must be a positive finite number of inches.");
                    return;
                }
                completed = runOpMode(new TestsLine(followerFunction, distance.get()));
                break;
            case CURVED:
                if (!followerConfigured) {
                    abort("Curve Test requires a configured algorithm, localizer, and drivetrain.");
                    return;
                }
                if (!(distance.get() > 0.0) || !Double.isFinite(distance.get())) {
                    abort("Test distance must be a positive finite number of inches.");
                    return;
                }
                completed = runOpMode(new TestsCurve(followerFunction, distance.get()));
                break;
            case INTERPOLATION_CURVED:
                if (!followerConfigured) {
                    abort("Interpolation Test requires a configured algorithm, localizer, and drivetrain.");
                    return;
                }
                if (!(distance.get() > 0.0) || !Double.isFinite(distance.get())) {
                    abort("Test distance must be a positive finite number of inches.");
                    return;
                }
                completed = runOpMode(new TestsInterpolation(followerFunction, distance.get()));
                break;
            case LOCALIZATION:
                if (!drivetrain || !localizer) {
                    abort("Localization Test requires a configured drivetrain and localizer.");
                    return;
                }
                completed = runOpMode(new TestsLocalization(drivetrainFunction, localizerFunction));
                break;
            case ODOMETRY:
                if (!drivetrain || !localizer) {
                    abort("Odometry Test requires a configured drivetrain and localizer.");
                    return;
                }
                completed = runOpMode(new TestsOdometry(drivetrainFunction, localizerFunction));
                if (!completed)
                    abort("Failed odometry test. Please check your odometry pods and ensure they are functioning correctly.");
                break;
            case POSE:
                if (!localizer) {
                    abort("Pose Test requires a configured localizer.");
                    return;
                }
                completed = runOpMode(new TestsPose(localizerFunction));
                break;
            case DRIVING:
                if (!drivetrain) {
                    abort("Driving Test requires a configured drivetrain.");
                    return;
                }
                completed = runOpMode(new TestsDriving(drivetrainFunction));
                break;
        }

        result("Completed", completed);
    }
}

class TestsHold extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;

    public TestsHold(Function<HardwareMap, Follower> followerFunction) {
        super("Hold Test", "Tests the Follower's ability to hold a position.", true);
        this.followerFunction = followerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());
        Thread.sleep(1000);
        waitForStart();
        follower.setPose(Pose.zero());
        follower.update();
        follower.hold(Pose.zero());
        while (opModeIsActive()) {
            follower.update();
        }
        follower.stop();
        return true;
    }
}

class TestsLine extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;
    double distance;

    public TestsLine(Function<HardwareMap, Follower> followerFunction, double distance) {
        super("Line Test", "Tests the Follower's ability to follow a line.", true);
        this.followerFunction = followerFunction;
        this.distance = distance;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());

        double pathDistance = this.distance;
        boolean forward = true;

        Path path1 = line(Pose.zero(), new Pose(pathDistance,0, 0)).constant(0);
        Path path2 = line(new Pose(pathDistance,0, 0), Pose.zero()).constant(0);

        Thread.sleep(1000);
        waitForStart();
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(path1);

        while (opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }
        follower.stop();
        return true;
    }
}

class TestsCurve extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;
    double distance;

    public TestsCurve(Function<HardwareMap, Follower> followerFunction, double distance) {
        super("Curve Test", "Tests the Follower's ability to follow a curve.", true);
        this.followerFunction = followerFunction;
        this.distance = distance;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());

        double curveDistance = this.distance;
        boolean forward = true;

        Path path1 = curve(Pose.zero(), new Pose(curveDistance,0), new Pose(curveDistance,curveDistance)).tangent();
        Path path2 = curve(new Pose(curveDistance,curveDistance), new Pose(curveDistance,0), Pose.zero()).tangent();

        Thread.sleep(1000);
        waitForStart();
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(path1);

        while (opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }
        follower.stop();
        return true;
    }
}

class TestsInterpolation extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;
    double distance;

    public TestsInterpolation(Function<HardwareMap, Follower> followerFunction, double distance) {
        super("Interpolation Curve Test", "Tests the Follower's ability to follow a curve with several interpolations.", true);
        this.followerFunction = followerFunction;
        this.distance = distance;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());

        double interpolationDistance = this.distance;
        boolean forward = true;

        Path path1 = curve(Pose.zero(), new Pose(interpolationDistance,0), new Pose(interpolationDistance,interpolationDistance)).heading((curve, t) -> Math.PI);
        Path path2 = curve(new Pose(interpolationDistance,interpolationDistance), new Pose(interpolationDistance,0), Pose.zero()).heading(Interpolator.piecewise().until(0.5, Interpolator.tangent).until(1.0, Interpolator.constant(0)));

        Thread.sleep(1000);
        waitForStart();
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(path1);

        while (opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }
        follower.stop();
        return true;
    }
}

class TestsLocalization extends TuningOpMode<Boolean> {
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    Function<HardwareMap, Localizer> localizerFunction;

    public TestsLocalization(Function<HardwareMap, Drivetrain> drivetrainFunction, Function<HardwareMap, Localizer> localizerFunction) {
        super("Localization Test", "Verifies localization and manual control.", true);
        this.drivetrainFunction = drivetrainFunction;
        this.localizerFunction = localizerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());

        while (opModeIsActive()) {
            drivetrain.drive(new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x), true);
            localizer.update();
            telemetry.addData("Pose", localizer.pose());
            telemetry.update();
        }
        drivetrain.stop();
        return true;
    }
}

class TestsOdometry extends TuningOpMode<Boolean> {
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    Function<HardwareMap, Localizer> localizerFunction;

    public enum Test {
        FORWARD,
        LEFT,
        TURN,
        IDLE
    }

    private Test test = Test.FORWARD;
    public static double POWER = 0.5;

    double totalHeading;
    double prevHeading;

    private final ElapsedTime timer = new ElapsedTime();

    private boolean passedX = false;
    private boolean passedY = false;
    private boolean passedHeading = false;

    public TestsOdometry(Function<HardwareMap, Drivetrain> drivetrainFunction, Function<HardwareMap, Localizer> localizerFunction) {
        super("Localization Test", "Verifies localization and manual control.", true);
        this.drivetrainFunction = drivetrainFunction;
        this.localizerFunction = localizerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());

        timer.reset();

        while (opModeIsActive()) {
            localizer.update();

            if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero().y() || localizer.pose().heading() != Pose.zero().heading()) {
                double currentHeading = localizer.pose().heading();
                totalHeading += Angle.normalizeSigned(currentHeading - prevHeading);
                prevHeading = currentHeading;
            }

            switch (test) {
                case FORWARD:
                    drivetrain.drive(new DrivePowers(POWER, 0, 0), false);

                    if (timer.seconds() > 1) {
                        test = Test.LEFT;
                        timer.reset();
                    }
                    break;
                case LEFT:
                    drivetrain.drive(new DrivePowers(0, POWER, 0), false);

                    if (timer.seconds() > 1) {
                        test = Test.TURN;
                        timer.reset();

                        prevHeading = localizer.pose().heading();
                        totalHeading = 0;
                    }
                    break;
                case TURN:
                    drivetrain.drive(new DrivePowers(0, 0, POWER), false);

                    if (timer.seconds() > 1) {
                        test = Test.IDLE;
                        timer.reset();
                    }
                    break;
                case IDLE:
                    drivetrain.drive(new DrivePowers(0, 0, 0), true);

                    telemetry.addData("Test", "Completed");

                    Pose pose = localizer.pose();

                    if (pose.x() < 0)
                        telemetry.addData("xPod Direction", "Flipped");
                    else if (pose.x() < 2)
                        telemetry.addData("xPod Resolution", "Too high");
                    else if (pose.x() > 144)
                        telemetry.addData("xPod Resolution", "Too low");
                    else {
                        telemetry.addData("xPod", "Good");
                        passedX = true;
                    }

                    if (pose.y() < 0)
                        telemetry.addData("yPod Direction", "Flipped");
                    else if (pose.y() < 2)
                        telemetry.addData("yPod Resolution", "Too high");
                    else if (pose.y() > 144)
                        telemetry.addData("yPod Resolution", "Too low");
                    else {
                        telemetry.addData("yPod", "Good");
                        passedY = true;
                    }

                    if (totalHeading < 0)
                        telemetry.addData("Heading Direction", "Flipped");
                    else if (totalHeading < 0.02)
                        telemetry.addData("Heading Resolution", "Too high");
                    else if (totalHeading > 2 * Math.PI)
                        telemetry.addData("Heading Resolution", "Too low");
                    else {
                        telemetry.addData("Heading", "Good");
                        passedHeading = true;
                    }

                    break;
            }

            telemetry.addData("Pose", localizer.pose());
            telemetry.update();
        }
        drivetrain.stop();
        return passedX && passedY && passedHeading;
    }
}

class TestsDriving extends TuningOpMode<Boolean> {
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public TestsDriving(Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Driving Test", "Tests raw drivetrain control without localization.", true);
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            drivetrain.drive(new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x), true);
        }
        drivetrain.stop();
        return true;
    }
}

class TestsPose extends TuningOpMode<Boolean> {
    Function<HardwareMap, Localizer> localizerFunction;

    public TestsPose(Function<HardwareMap, Localizer> localizerFunction) {
        super("Pose Test", "Verifies localizer output without a drivetrain.", true);
        this.localizerFunction = localizerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        while (opModeIsActive()) {
            localizer.update();
            telemetry.addData("Pose", localizer.pose());
            telemetry.update();
        }
        return true;
    }
}


