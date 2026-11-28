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

@Autonomous(name = "Pedropath1Red")
public class pedropath1red extends OpMode {

    private static final double FEEDER_START_DELAY_SECONDS = 1.0;
    private static final double FEEDING_TIME_SECONDS = 7.5;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(11.12, 103.46, 109);
    private final Pose firstPointPose = poseFactory.of(29.59, 28.96, 109);
    private final Pose collectionPointPose = poseFactory.of(6.39, 8.09, 80);
    private final Pose activationPose = poseFactory.of(56.9, 14.6, 90);
    private final Pose path4ControlPoint = poseFactory.of(39.95, 50.17, 0);
    private final Pose path4EndPose = poseFactory.of(8.94, 108.12, 0);

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
        TO_COLLECTION_POINT,
        TO_ACTIVATION_POINT,
        WAITING_TO_FEED,
        FEEDING,
        TO_PATH4_END,
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

        path1 = line(startPose, firstPointPose)
                .linear(startPose, firstPointPose);
        path2 = line(firstPointPose, collectionPointPose)
                .linear(firstPointPose, collectionPointPose);
        path3 = line(collectionPointPose, activationPose)
                .linear(collectionPointPose, activationPose);
        path4 = curve(activationPose, path4ControlPoint, path4EndPose)
                .linear(activationPose, path4EndPose);

        follower.setPose(startPose);
        feeder.stop();

        telemetry.addLine("Pedropath1Red ready");
        telemetry.addData("Start", "(%.2f, %.2f) | %.0f°",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.addData("Feeder activation", "(%.2f, %.2f) | %.0f°",
                activationPose.x(), activationPose.y(), Math.toDegrees(activationPose.heading()));
        telemetry.addData("Path 4 end", "(%.2f, %.2f) | %.0f°",
                path4EndPose.x(), path4EndPose.y(), Math.toDegrees(path4EndPose.heading()));
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
                    state = AutoState.TO_COLLECTION_POINT;
                    follower.follow(path2);
                }
                break;

            case TO_COLLECTION_POINT:
                if (!follower.isBusy()) {
                    state = AutoState.TO_ACTIVATION_POINT;
                    follower.follow(path3);
                }
                break;

            case TO_ACTIVATION_POINT:
                if (!follower.isBusy()) {
                    follower.stop();
                    stateTimer.reset();
                    state = AutoState.WAITING_TO_FEED;
                }
                break;

            case WAITING_TO_FEED:
                if (stateTimer.seconds() >= FEEDER_START_DELAY_SECONDS) {
                    feeder.run();
                    stateTimer.reset();
                    state = AutoState.FEEDING;
                }
                break;

            case FEEDING:
                if (stateTimer.seconds() >= FEEDING_TIME_SECONDS) {
                    feeder.stop();
                    shooter.stop();
                    state = AutoState.TO_PATH4_END;
                    follower.follow(path4);
                }
                break;

            case TO_PATH4_END:
                if (!follower.isBusy()) {
                    follower.stop();
                    state = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                // Keep the intake on through the end of the autonomous route.
                break;
        }

        telemetry.addData("Route state", state);
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("Pose", "X %.2f | Y %.2f | H %.1f°",
                follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        if (state == AutoState.WAITING_TO_FEED) {
            telemetry.addData("Feeder starts in", "%.1f s",
                    Math.max(0.0, FEEDER_START_DELAY_SECONDS - stateTimer.seconds()));
        } else if (state == AutoState.FEEDING) {
            telemetry.addData("Feeding", "%.1f / %.1f s",
                    stateTimer.seconds(), FEEDING_TIME_SECONDS);
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
