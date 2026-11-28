package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

@Autonomous(name = "Pedropath1")
public class Pedropath1blue extends OpMode {

    private static final double STATIONARY_WAIT_SECONDS = 7.5;
    private static final double GARDEN_WAIT_SECONDS = 4.0;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Coordinates and headings supplied for this route (heading values are degrees).
    private final Pose startPose = poseFactory.of(130.0, 32.4, 106);
    private final Pose firstPointPose = poseFactory.of(102.6, 128.2, 106);
    private final Pose secondPointPose = poseFactory.of(83.6, 126.5, 90);
    private final Pose gardenPose = poseFactory.of(129.8, 131.5, 78);
    private final Pose gardenOffsetPose = poseFactory.of(133, 132, 80);
    private final Pose parkingCurveControlPose = poseFactory.of(106.0, 82.0, 0);
    private final Pose parkingPose = poseFactory.of(128.3, 32.5, 0);

    // The package also has a teamcode class named Follower, so explicitly use Pedro's.
    private com.pedropathing.follower.Follower follower;

    private Intake intake;
    private Shooter shooter;
    private Feeder feeder;
    private ElapsedTime stateTimer;

    private Path path1;
    private Path path2;
    private Path pathToGarden;
    private Path pathGardenOffset;
    private Path pathToParking;

    private enum AutoState {
        TO_POINT_1,
        TO_POINT_2,
        STATIONARY_AT_POINT_2,
        TO_GARDEN,
        TO_GARDEN_OFFSET,
        WAITING_AT_GARDEN,
        TO_PARKING,
        COMPLETE
    }

    private AutoState state;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
        stateTimer = new ElapsedTime();

        // Pedro 3 straight paths with heading interpolated between each pair of poses.
        path1 = line(startPose, firstPointPose)
                .linear(startPose, firstPointPose);
        path2 = line(firstPointPose, secondPointPose)
                .linear(firstPointPose, secondPointPose);
        pathToGarden = line(secondPointPose, gardenPose)
                .linear(secondPointPose, gardenPose);
        pathGardenOffset = line(gardenPose, gardenOffsetPose)
                .linear(gardenPose, gardenOffsetPose);
        pathToParking = curve(gardenOffsetPose, parkingCurveControlPose, parkingPose)
                .linear(gardenOffsetPose, parkingPose);

        follower.setPose(startPose);
        feeder.stop();

        telemetry.addLine("Pedropath1 ready");
        telemetry.addData("Start", "(%.2f, %.2f) | %.0f°",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.addData("Point 1", "(%.2f, %.2f) | %.0f°",
                firstPointPose.x(), firstPointPose.y(), Math.toDegrees(firstPointPose.heading()));
        telemetry.addData("Point 2", "(%.2f, %.2f) | %.0f°",
                secondPointPose.x(), secondPointPose.y(), Math.toDegrees(secondPointPose.heading()));
        telemetry.addData("Garden", "(%.2f, %.2f) | %.0f°",
                gardenPose.x(), gardenPose.y(), Math.toDegrees(gardenPose.heading()));
        telemetry.addData("Parking", "(%.2f, %.2f) | %.0f°",
                parkingPose.x(), parkingPose.y(), Math.toDegrees(parkingPose.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        intake.run();
        shooter.run(0.7);
        feeder.stop();
        state = AutoState.TO_POINT_1;
        follower.follow(path1);
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case TO_POINT_1:
                if (!follower.isBusy()) {
                    state = AutoState.TO_POINT_2;
                    follower.follow(path2);
                }
                break;

            case TO_POINT_2:
                if (!follower.isBusy()) {
                    // Freeze the drivetrain at point 2; intake, shooter, and feeder run.
                    follower.stop();
                    feeder.run();
                    stateTimer.reset();
                    state = AutoState.STATIONARY_AT_POINT_2;
                }
                break;

            case STATIONARY_AT_POINT_2:
                if (stateTimer.seconds() >= STATIONARY_WAIT_SECONDS) {
                    shooter.stop();
                    feeder.stop();
                    state = AutoState.TO_GARDEN;
                    follower.follow(pathToGarden);
                }
                break;

            case TO_GARDEN:
                if (!follower.isBusy()) {
                    follower.stop();
                    state = AutoState.TO_GARDEN_OFFSET;
                    follower.follow(pathGardenOffset);
                }
                break;

            case TO_GARDEN_OFFSET:
                if (!follower.isBusy()) {
                    follower.stop();
                    stateTimer.reset();
                    state = AutoState.WAITING_AT_GARDEN;
                }
                break;

            case WAITING_AT_GARDEN:
                if (stateTimer.seconds() >= GARDEN_WAIT_SECONDS) {
                    state = AutoState.TO_PARKING;
                    follower.follow(pathToParking);
                }
                break;

            case TO_PARKING:
                if (!follower.isBusy()) {
                    follower.stop();
                    state = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                // Intake and shooter remain on until STOP; stop() shuts everything down.
                break;
        }

        telemetry.addData("Route state", state);
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("Pose", "X %.2f | Y %.2f | H %.1f°",
                follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        if (state == AutoState.STATIONARY_AT_POINT_2) {
            telemetry.addData("Point 2 stationary; feeder on", "%.1f / %.1f s",
                    stateTimer.seconds(), STATIONARY_WAIT_SECONDS);
        }
        if (state == AutoState.WAITING_AT_GARDEN) {
            telemetry.addData("Garden wait", "%.1f / %.1f s",
                    stateTimer.seconds(), GARDEN_WAIT_SECONDS);
        }
        telemetry.update();
    }

    private void stopMechanisms() {
        intake.stop();
        shooter.stop();
        feeder.stop();
        if (follower != null) {
            follower.stop();
        }
    }

    @Override
    public void stop() {
        stopMechanisms();
    }
}
