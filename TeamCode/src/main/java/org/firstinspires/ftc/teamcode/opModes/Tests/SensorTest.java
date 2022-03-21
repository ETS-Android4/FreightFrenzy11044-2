package org.firstinspires.ftc.teamcode.opModes.Tests;

import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.LINE_SENSOR_BACK;
//import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.LINE_SENSOR_MID;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDIO;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot21;

@Config
@TeleOp(group = "Sensor")
public class SensorTest extends LinearOpMode {
    private AnalogInput input;
    private AnalogInput backline, midline;
//    public static double midvalue = 0.35;
    public static double backvalue = 0.35;

    @Override
    public void runOpMode() {
        Robot21 R = new Robot21();
        R.attachGamepads(gamepad1, gamepad2);
        R.init(this);
//        input = hardwareMap.get(AnalogInput.class, "sensor_range");
//        backline = hardwareMap.get(AnalogInput.class, LINE_SENSOR_BACK);
//        midline = hardwareMap.get(AnalogInput.class, LINE_SENSOR_MID);

        waitForStart();

        ElapsedTime t = new ElapsedTime();
        while(opModeIsActive()) {
            R.control(false, t.milliseconds());
            R.intake.update();
//            telemetry.addData("Voltage: ", input.getVoltage());
            telemetry.addData("isFull", R.intake.isFull());
//            telemetry.addData("isLineMid", midline.getVoltage() < midvalue);
//            telemetry.addData("isLineBack", backline.getVoltage() < backvalue);
//            telemetry.addData("BackVoltage", backline.getVoltage());
            telemetry.addData("TiltVel", R.imu.tiltVelocity());
            telemetry.addData("tilthead", R.imu.getTiltHeading());
            telemetry.update();
        }
    }

}