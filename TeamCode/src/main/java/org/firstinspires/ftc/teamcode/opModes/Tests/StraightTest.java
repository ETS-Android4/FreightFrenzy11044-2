package org.firstinspires.ftc.teamcode.opModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.misc.DrivePower;
import org.firstinspires.ftc.teamcode.pathFollowing.Path;
import org.firstinspires.ftc.teamcode.pathFollowing.PurePursuitTracker;

import java.util.ArrayList;
import java.util.List;

//@Config
@Autonomous
public class StraightTest extends LinearOpMode {
    Path path = new Path(true);
    public static double k = 0.1;
    public static double lookaheadDistance = 5;
    public static double maxVel = 30;
    public static double maxAccel = 30;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot21 R = new Robot21();
        R.init(this);
        R.localizer.setStartPose(new Pose2d(0, 0, Math.toRadians(160)));
        PurePursuitTracker tracker = new PurePursuitTracker(R.drivetrainTank, R.localizer, this);
//        path.generatePath(hardwareMap, org.firstinspires.ftc.teamcode.R.raw.straighttest);
//        path.initializePath(maxVel, maxAccel, k, 5);
//
//        List<Path> paths = new ArrayList<>();
//        paths.add(path);

//        tracker.setRobotTrack(16.3);
//        tracker.setPaths(paths, lookaheadDistance);
//        tracker.setPath(0);

        waitForStart();

        tracker.rotateTo(-160);
        R.drivetrainTank.setPowerSimple(0,0);
    }
}
