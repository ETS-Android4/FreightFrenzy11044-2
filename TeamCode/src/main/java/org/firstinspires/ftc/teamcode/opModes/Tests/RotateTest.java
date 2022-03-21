package org.firstinspires.ftc.teamcode.opModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot21;
import org.firstinspires.ftc.teamcode.pathFollowing.PurePursuitTracker;

@Disabled
//@Config
@TeleOp
public class RotateTest extends LinearOpMode {
    public static double target = 90;


    @Override
    public void runOpMode() throws InterruptedException {
        Robot21 R = new Robot21();
        R.init(this);
        PurePursuitTracker tracker = new PurePursuitTracker(R.drivetrainTank, R.localizer, this, R.lift, R.imu);

        waitForStart();

        while (!isStopRequested()) {
            tracker.rotateTo(target);
        }
    }
}
