package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "Pedropath1")
public class Pedropath1blue extends OpMode {

    private static final double STATIONARY_WAIT_SECONDS = 5.0;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Coordinates and headings supplied for this route (heading values are degrees).
    private final Pose startPose = poseFactory.of(130.0, 32.4, 106);
    private final Pose firstPointPose = poseFactory.of(102.6, 128.2, 106);
    private final Pose secondPointPose = poseFactory.of(83.6, 126.5, 90);
    private final Pose thirdPointPose = poseFactory.of(102.42, 129.36, -174);
    private final Pose fourthPointPose = poseFactory.of(102.4, 129.4, 106);

    // The package also has a teamcode class named Follower, so explicitly use Pedro's.
    private com.pedropathing.follower.Follower follower;

    private Intake intake;
    private Shooter shooter;
    private Feeder feeder;
    private ElapsedTime stateTimer;

    private Path path1;
    private Path path2;
    private Path path3;
    private Path path4;

    private enum AutoState {
        TO_POINT_1,
        TO_POINT_2,
        STATIONARY_AT_POINT_2,
        TO_POINT_3,
        TO_POINT_4,
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
        path3 = line(secondPointPose, thirdPointPose)
                .linear(secondPointPose, thirdPointPose);
        path4 = line(thirdPointPose, fourthPointPose)
                .linear(thirdPointPose, fourthPointPose);

        follower.setPose(startPose);
        feeder.stop();

        telemetry.addLine("Pedropath1 ready");
        telemetry.addData("Start", "(%.2f, %.2f) | %.0f°",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.addData("Point 1", "(%.2f, %.2f) | %.0f°",
                firstPointPose.x(), firstPointPose.y(), Math.toDegrees(firstPointPose.heading()));
        telemetry.addData("Point 2", "(%.2f, %.2f) | %.0f°",
                secondPointPose.x(), secondPointPose.y(), Math.toDegrees(secondPointPose.heading()));
        telemetry.addData("Point 3", "(%.2f, %.2f) | %.0f°",
                thirdPointPose.x(), thirdPointPose.y(), Math.toDegrees(thirdPointPose.heading()));
        telemetry.addData("Point 4", "(%.2f, %.2f) | %.0f°",
                fourthPointPose.x(), fourthPointPose.y(), Math.toDegrees(fourthPointPose.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        intake.run();
        shooter.run();
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
                    feeder.stop();
                    state = AutoState.TO_POINT_3;
                    follower.follow(path3);
                }
                break;

            case TO_POINT_3:
                if (!follower.isBusy()) {
                    state = AutoState.TO_POINT_4;
                    follower.follow(path4);
                }
                break;

            case TO_POINT_4:
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
