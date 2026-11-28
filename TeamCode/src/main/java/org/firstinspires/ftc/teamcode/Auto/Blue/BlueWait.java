package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "Blue Wait", group = "Blues")
public class BlueWait extends OpMode {

    private static final int APRILTAG_PIPELINE = 8;
    private static final double FEEDING_TIME_SECONDS = 9;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(132.4, 34.6, 270);
    private final Pose shootingPose = poseFactory.of(81.46, 17, 270);

    private com.pedropathing.follower.Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Feeder feeder;
    private Limelight3A limelight;
    private ElapsedTime stateTimer;

    private Path pathToShootingPose;
    private Path pathBackToStart;

    private enum AutoState {
        DRIVING_TO_SHOOTING_POSE,
        WAITING_FOR_APRILTAG,
        FEEDING,
        RETURNING_TO_START,
        COMPLETE
    }

    private AutoState state;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        stateTimer = new ElapsedTime();

        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        pathToShootingPose = line(startPose, shootingPose)
                .linear(startPose, shootingPose);
        pathBackToStart = line(shootingPose, startPose)
                .linear(shootingPose, startPose);

        follower.setPose(startPose);
        feeder.stop();

        telemetry.addLine("Blue Wait ready");
        telemetry.addData("Start", "(%.2f, %.2f) | %.0f°",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.addData("Shooting pose", "(%.2f, %.2f) | %.0f°",
                shootingPose.x(), shootingPose.y(), Math.toDegrees(shootingPose.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        intake.run();
        shooter.run();
        feeder.stop();

        state = AutoState.DRIVING_TO_SHOOTING_POSE;
        follower.follow(pathToShootingPose);
    }

    @Override
    public void loop() {
        follower.update();

        boolean seesAprilTag = seesAprilTag();

        switch (state) {
            case DRIVING_TO_SHOOTING_POSE:
                if (!follower.isBusy()) {
                    follower.stop();
                    state = AutoState.WAITING_FOR_APRILTAG;
                }
                break;

            case WAITING_FOR_APRILTAG:
                if (seesAprilTag) {
                    feeder.run();
                    stateTimer.reset();
                    state = AutoState.FEEDING;
                }
                break;

            case FEEDING:
                if (stateTimer.seconds() >= FEEDING_TIME_SECONDS) {
                    feeder.stop();
                    intake.stop();
                    shooter.stop();
                    state = AutoState.RETURNING_TO_START;
                    follower.follow(pathBackToStart);
                }
                break;

            case RETURNING_TO_START:
                if (!follower.isBusy()) {
                    follower.stop();
                    state = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                break;
        }

        telemetry.addData("Route state", state);
        telemetry.addData("AprilTag visible", seesAprilTag);
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("Pose", "X %.2f | Y %.2f | H %.1f°",
                follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        if (state == AutoState.FEEDING) {
            telemetry.addData("Feeding", "%.1f / %.1f s",
                    stateTimer.seconds(), FEEDING_TIME_SECONDS);
        }
        telemetry.update();
    }

    private boolean seesAprilTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return false;
        }

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int tagId = tag.getFiducialId();
            if (tagId == 38 || tagId == 39 || tagId == 40 || tagId == 41) {
                return true;
            }
        }

        return false;
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
