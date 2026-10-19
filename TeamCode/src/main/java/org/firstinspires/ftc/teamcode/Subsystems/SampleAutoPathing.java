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

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE POLLEN/NECTAR

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_ENDPOS
    }

    private PathState pathState;

    // Pedro 3: build poses with a PoseFactory (degrees here)
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(56, 8, 90);
    private final Pose shootPose = p.of(0, 0, 0); // PLACEHOLDER VALUES
    private final Pose endPose = p.of(0, 0, 0); // PLACEHOLDER VALUES

    private Path driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = line(startPose, shootPose)
                .linear(startPose, shootPose);

        driveShootPosEndPos = line(shootPose, endPose)
                .linear(shootPose, endPose);
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.follow(driveStartPosShootPos);
                setPathState(PathState.SHOOT_PRELOAD); // reset the timer & make new state
                break;

            case SHOOT_PRELOAD:
                // check: is the follower done with its path?
                // check: has 5 seconds elapsed?
                if (!follower.isBusy() && pathTimer.seconds() > 5) {
                    // TODO add shooting logic here(flywheels)
                    follower.follow(driveShootPosEndPos);
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
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
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
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