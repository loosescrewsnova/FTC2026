
package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.PollenPusher;
import org.firstinspires.ftc.teamcode.Subsystems.shooter.Shooter;

import static com.pedropathing.api.Paths.*;

@Autonomous(name = "Pedropath1")

public class Pedropath1 extends OpMode {

    // =========================================
    // SUBSYSTEMS
    // =========================================

    private Follower follower;

    private Intake intake;
    private Shooter shooter;
    private Feeder feeder;
    private PollenPusher pusher;

    private ElapsedTime timer = new ElapsedTime();

    // =========================================
    // COORDINATES
    // =========================================

    private final PoseFactory p = PoseFactory.degrees();

    // Starting position
    private final Pose startPose = p.of(56, 8, 90);

    // Path 1 endpoint: Hive approach
    private final Pose hivePosition = p.of(57.21, 9.85, 90);

    // Red Alliance hive center
    private static final double HIVE_X = 57.77;
    private static final double HIVE_Y = 69.7;

    // Path 2 endpoint: Ball collection
    private final Pose collectionPosition = p.of(5.5, 1.35, 0);

    // Path 3 endpoint: Shooting position
    private final Pose shootingPosition = p.of(57.21, 9.85, 0);

    // Heading that makes the back face the hive
    private double hiveHeading;

    // =========================================
    // PATHS
    // =========================================

    private Path path1;
    private Path path2;
    private Path path3;

    // =========================================
    // STATES
    // =========================================

    private enum State {
        PATH1,
        PATH2,
        PATH3,
        WAIT_TO_SHOOT,
        FEEDING,
        FINISHED
    }

    private State state;

    // =========================================
    // TIMING
    // =========================================

    private static final double SHOOTER_DELAY = 5.0;
    private static final double FEED_TIME = 3.0;

    // =========================================
    // INITIALIZATION
    // =========================================

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
        pusher = new PollenPusher(hardwareMap);

        // Calculate direction toward the hive center

        double angleToHive = Math.atan2(
                HIVE_Y - hivePosition.y(),
                HIVE_X - hivePosition.x()
        );

        // Add 180 degrees so the back faces the hive

        hiveHeading = Math.toDegrees(angleToHive) + 180.0;

        hiveHeading = wrapDegrees(hiveHeading);

        // Pose at the hive with calculated heading

        Pose hiveFacingPose = p.of(
                hivePosition.x(),
                hivePosition.y(),
                hiveHeading
        );

        // =====================================
        // BUILD PATH 1: START -> HIVE
        // =====================================

        path1 = line(startPose, hiveFacingPose)
                .linear(startPose, hiveFacingPose);

        // =====================================
        // BUILD PATH 2: HIVE -> COLLECTION
        // =====================================

        Pose collectionPose = p.of(
                collectionPosition.x(),
                collectionPosition.y(),
                hiveHeading
        );

        path2 = line(hiveFacingPose, collectionPose)
                .constant(hiveFacingPose);

        // =====================================
        // BUILD PATH 3: COLLECTION -> SHOOTING
        // =====================================

        path3 = line(collectionPose, shootingPosition)
                .linear(collectionPose, shootingPosition);

        // Set starting localization pose

        follower.setPose(startPose);

        telemetry.addLine("Pedropath1 Ready");
        telemetry.addData("Hive Heading", hiveHeading);
        telemetry.addData("Start X", startPose.x());
        telemetry.addData("Start Y", startPose.y());

        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {

        state = State.PATH1;

        timer.reset();

        follower.follow(path1);
    }

    // =========================================
    // MAIN AUTONOMOUS LOOP
    // =========================================

    @Override
    public void loop() {

        // Update Pedro Pathing

        follower.update();

        switch (state) {

            // ---------------------------------
            // PATH 1: START -> HIVE
            // ---------------------------------

            case PATH1:

                if (!follower.isBusy()) {

                    // Start collecting balls

                    intake.run();

                    // Begin Path 2

                    follower.follow(path2);

                    state = State.PATH2;
                    timer.reset();
                }

                break;

            // ---------------------------------
            // PATH 2: HIVE -> COLLECTION
            // ---------------------------------

            case PATH2:

                if (!follower.isBusy()) {

                    // Stop intake after collection

                    intake.stop();

                    // Start shooter during Path 3

                    shooter.run();

                    // Begin Path 3

                    follower.follow(path3);

                    state = State.PATH3;
                    timer.reset();
                }

                break;

            // ---------------------------------
            // PATH 3: COLLECTION -> SHOOTING
            // ---------------------------------

            case PATH3:

                if (!follower.isBusy()) {

                    // Wait for shooter to get ready

                    timer.reset();

                    state = State.WAIT_TO_SHOOT;
                }

                break;

            // ---------------------------------
            // WAIT FOR SHOOTER
            // ---------------------------------

            case WAIT_TO_SHOOT:

                if (timer.seconds() >= SHOOTER_DELAY) {

                    feeder.run();

                    timer.reset();

                    state = State.FEEDING;
                }

                break;

            // ---------------------------------
            // FEED COLLECTED BALLS
            // ---------------------------------

            case FEEDING:

                if (timer.seconds() >= FEED_TIME) {

                    stopMechanisms();

                    state = State.FINISHED;
                }

                break;

            // ---------------------------------
            // FINISHED
            // ---------------------------------

            case FINISHED:

                stopMechanisms();

                telemetry.addLine("AUTONOMOUS COMPLETE");

                break;
        }

        // =====================================
        // TELEMETRY
        // =====================================

        telemetry.addData("Path State", state.toString());

        telemetry.addData(
                "X Position",
                follower.pose().x()
        );

        telemetry.addData(
                "Y Position",
                follower.pose().y()
        );

        telemetry.addData(
                "Heading (deg)",
                Math.toDegrees(follower.pose().heading())
        );

        telemetry.addData(
                "State Timer",
                timer.seconds()
        );

        telemetry.update();
    }

    // =========================================
    // STOP SUBSYSTEMS
    // =========================================

    private void stopMechanisms() {

        intake.stop();
        shooter.stop();
        feeder.stop();
        pusher.stop();
    }

    @Override
    public void stop() {

        stopMechanisms();
    }

    // =========================================
    // MATH
    // =========================================

    private double wrapDegrees(double angle) {

        while (angle > 180) {
            angle -= 360;
        }

        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }
}
