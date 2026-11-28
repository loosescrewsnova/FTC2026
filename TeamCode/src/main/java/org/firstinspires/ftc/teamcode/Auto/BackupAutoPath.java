package org.firstinspires.ftc.teamcode.Auto;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;

@Autonomous(name = "BackupAutoPath")
public class BackupAutoPath extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose1 = p.of(128.9, 29.0, 120);
    private final Pose park1 = p.of(115.0, 53.5, 120);

    private final Pose getStartPose1 = p.of(115.0, 53.5, 139);
    private final Pose park2 = p.of(129.37,40.85 , 139);

    private Path goToPark1;
    private Path goToPark2;

    private int step = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose1);

        goToPark1 = line(startPose1, park1).linear(startPose1, park1);

        goToPark2 = line(getStartPose1, park2).linear(getStartPose1,park2);
    }
    @Override
    public void start() {
        step = 1;
        follower.follow(goToPark1);
    }
    private Path park() {
        return line(startPose1, park1).linear(startPose1, park1);
    }


    @Override
    public void loop() {
        follower.update();

        switch (step) {
            case 1:
                if (!follower.isBusy()) {
                    follower.hold(getStartPose1);
                    step = 2;
                }
                break;

            case 2:

                if (!follower.isBusy()) {
                    follower.follow(goToPark2);
                    step = 3;
                }
                break;

            case 3:
                // Finished at park2
                if (!follower.isBusy()) {
                    step = 4;
                }
                break;
        }
    }
}
