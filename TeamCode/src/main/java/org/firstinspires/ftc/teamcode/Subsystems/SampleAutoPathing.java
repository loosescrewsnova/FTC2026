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
public class SampleAutoPathing extends OpMode {
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    private Intake intakeMotor;
    private Shooter shooterMotor;
    private Feeder feederServo;

    private static final double INTAKE_TIME = 4.0; //how long the intake will run for at the end of path 1
    private static final double SHOOT_TIME = 3.0; //how long the shooter has to run to get to top speed before filtering
    private static final double FEED_TIME = 4.0; //how long the feeder will feed the balls


    public enum PathState {
        // START POSITION to INTAKE POSITION
        // INTAKE
        // INTAKE POSITION TO SHOOT POSITION
        //SHOOT!
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE POLLEN/NECTAR

        DRIVE_STARTPOS_INTAKE_POS,
        INTAKE_PRELOAD,
        DRIVE_INTAKEPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        FEED_PRELOAD,
        DONE
    }

    private PathState pathState;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(56, 8, 90);
    private final Pose intakePose = p.of(82.3, 32.73, 45);
    private final Pose shootPose = p.of(0, 0, 0); // PLACEHOLDER VALUES

    private Path driveStartPosIntakePos, driveIntakePosShootPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosIntakePos = line(startPose, intakePose).linear(startPose, intakePose);

        driveIntakePosShootPos = line(intakePose, shootPose).linear(intakePose, shootPose);
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_INTAKE_POS:
                //path will start in start() once
                if (!follower.isBusy()) {
                    intakeMotor.run();
                    setPathState(PathState.INTAKE_PRELOAD); // reset the timer & make new state
                }
                break;

            case INTAKE_PRELOAD:
                //follower.follow(driveIntakePosShootPos);
                if (!follower.isBusy() && pathTimer.seconds() > INTAKE_TIME) {
                    follower.follow(driveIntakePosShootPos);
                    setPathState(PathState.DRIVE_INTAKEPOS_SHOOTPOS); // reset the timer & make new state
                }
                break;

            case DRIVE_INTAKEPOS_SHOOTPOS:
                if (!follower.isBusy()) {
                    shooterMotor.run();
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

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
                    stopAll();
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
            default:
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
    public void init () {
        pathState = PathState.DRIVE_STARTPOS_INTAKE_POS;
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
        follower.follow(driveStartPosIntakePos); //first path once
        setPathState(PathState.DRIVE_STARTPOS_INTAKE_POS);
    }

    @Override
    public void loop () {
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