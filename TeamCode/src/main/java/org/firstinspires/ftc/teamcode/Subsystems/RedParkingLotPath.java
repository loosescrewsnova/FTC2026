package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.*;

@Autonomous
public class RedParkingLotPath extends OpMode {
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    private Intake intakeMotor;
    private Shooter shooterMotor;
    private Feeder feederServo;

    private static final double INTAKE_TIME = 4.0; //how long the intake will run for at the end of path 1
    private static final double SHOOT_TIME = 3.0; //how long the shooter has to run to get to top speed before filtering
    private static final double FEED_TIME = 4.0; //how long the feeder will feed the balls

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE POLLEN/NECTAR

        SHOOT_PRELOAD,
        FEED_PRELOAD,
        DRIVE_START_POS_INTAKE_POS,
        INTAKE_AT_INTAKE_POS,        // new
        DRIVE_INTAKE_POS_SHOOT_POS,
        SECOND_FEED_STAGE,
        DONE
    }

    private PathState pathState;


    // Pedro 3: build poses with a PoseFactory (degrees here)
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(8.263521150934617, 105.7521349401282, 270);
    private final Pose avoidHiveBaseControl = p.of(23.17, 22.78, 0);  // CONTROL POINT
    private final Pose intakePose = p.of(56.27, 16.18, 270);
    private final Pose shootPose = p.of(15.45, 10.02, 190);

    private Path driveStartPosIntakePos, driveIntakePosShootPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosIntakePos = curve(startPose, avoidHiveBaseControl, intakePose)
                .linear(startPose, intakePose);

        driveIntakePosShootPos = line(intakePose, shootPose)
                .linear(intakePose, shootPose);
    }

    public void statePathUpdate() {
        switch (pathState) {

            case SHOOT_PRELOAD:
                // check: is the follower done with its path?
                // check: has 5 seconds elapsed?
                if (!follower.isBusy() && pathTimer.seconds() > SHOOT_TIME) {
                    feederServo.run();
                    setPathState(PathState.FEED_PRELOAD);
                }
                break;
            case FEED_PRELOAD:
                if (!follower.isBusy() && pathTimer.seconds() > FEED_TIME) {
                    feederServo.stop();
                    setPathState(PathState.DRIVE_START_POS_INTAKE_POS);
                }
                break;

            case DRIVE_START_POS_INTAKE_POS:
                follower.follow(driveStartPosIntakePos);          // start path 1 once
                setPathState(PathState.INTAKE_AT_INTAKE_POS);
                break;

            case INTAKE_AT_INTAKE_POS:
                // wait until path 1 is done AND the intake has had INTAKE_TIME
                if (!follower.isBusy() && pathTimer.seconds() > INTAKE_TIME) {
                    follower.follow(driveIntakePosShootPos);      // start path 2 once
                    setPathState(PathState.DRIVE_INTAKE_POS_SHOOT_POS);
                }
                break;

            case DRIVE_INTAKE_POS_SHOOT_POS:
                if (!follower.isBusy()) {                         // just wait, no follow() here
                    feederServo.run();
                    setPathState(PathState.SECOND_FEED_STAGE);
                }
                break;

            case SECOND_FEED_STAGE:
                if (!follower.isBusy() && pathTimer.seconds() > FEED_TIME) {
                    feederServo.stop();
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                stopAll();
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset();
    }

    private void stopAll() {
        intakeMotor.stop();
        shooterMotor.stop();
        feederServo.stop();
    }

    @Override
    public void init() {
        pathState = PathState.SHOOT_PRELOAD;
        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        follower = Constants.create(hardwareMap);
        intakeMotor = new Intake(hardwareMap);
        shooterMotor = new Shooter(hardwareMap);
        feederServo = new Feeder(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start () {
        opModeTimer.reset();
        setPathState(PathState.SHOOT_PRELOAD);
        shooterMotor.run();
        intakeMotor.run();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Path time", pathTimer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAll();
    }
}