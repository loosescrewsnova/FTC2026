package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.*;

@Autonomous
public class SampleAutoPathing extends OpMode {
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    private DcMotor intakeMotor, shooterMotor, feederMotor;

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
        DONE
    }

    private PathState pathState;
    private boolean path1Done = false;
    private boolean path2done = false;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(56, 8, 90);
    private final Pose intakePose = p.of(82.3, 32.73, 45); // PLACEHOLDER VALUES
    private final Pose shootPose = p.of(0, 0, 0); // PLACEHOLDER VALUES

    private Path driveStartPosIntakePos, driveIntakePosShootPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosIntakePos = line(startPose, intakePose)
                .linear(startPose, intakePose);

        driveIntakePosShootPos = line(intakePose, shootPose)
                .linear(intakePose, shootPose);
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_INTAKE_POS:
                follower.follow(driveStartPosIntakePos);
                setPathState(PathState.INTAKE_PRELOAD); // reset the timer & make new state
                if (!follower.isBusy()) {
                    path1Done = true;



                }

            case SHOOT_PRELOAD:
                // check: is the follower done with its path?
                // check: has 5 seconds elapsed?
                if (!follower.isBusy() && pathTimer.seconds() > 5) {
                    // TODO add shooting logic here(flywheels)
                    follower.follow(driveIntakePosShootPos);
                    setPathState(PathState.DRIVE_INTAKEPOS_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKEPOS_SHOOTPOS:
                // all done?
                if (!follower.isBusy()) {
                    telemetry.addLine("Done Path 2");
                }
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

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_INTAKE_POS;
        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        follower = Constants.create(hardwareMap);
        // TODO add in any other init mechanisms (limelight, flywheels, etc.)

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(pathState);
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
}