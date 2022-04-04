package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.INTAKE_SENSOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.RobotMotor;

public class Intake extends RobotPart{
    public RobotMotor motor;
    private DigitalChannel input;
    public boolean isFull = true;
    private double averageVoltage = 0;
    public static double intakeMaxTime = 4000;
    public static double intakePwr = 0.9;

    @Override
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = new RobotMotor(opMode.hardwareMap.get(DcMotorEx.class, HardwareConfig.INTAKE_MOTOR));
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        input = opMode.hardwareMap.get(DigitalChannel.class, INTAKE_SENSOR);
        input.setMode(DigitalChannel.Mode.INPUT);
        opMode.telemetry.addData("Intake initialized!", null);
    }

    @Override
    public void control(Gamepad gamepad) {
//        update();
        if (Math.abs(gamepad.left_stick_y) >= 0.1) {
            if (gamepad.left_bumper)
                motor.setPower(-gamepad.left_stick_y * 0.7);
            else
                motor.setPower(-gamepad.left_stick_y);
        } else {
            motor.setPower(0);
        }

//        opMode.telemetry.addData("inp voltage", averageVoltage);
//        opMode.telemetry.addData("isFull", isFull);
    }

    public void autocontrol(double pwr, double maxTime) {
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < maxTime && !opMode.isStopRequested()) {
            motor.setPower(pwr);
        }
        motor.setPower(0);
    }

    public void autoIn() {
        update();
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < intakeMaxTime && !opMode.isStopRequested() && !isFull) {
            motor.setPower(-intakePwr);
            update();
        }
        t = new ElapsedTime();
        while (t.milliseconds() < 200)
            motor.setPowerClassic(-intakePwr);
        motor.setPower(0);
    }

    public void autoIn(double power) {
        update();
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < intakeMaxTime && !opMode.isStopRequested() && !isFull) {
            motor.setPower(-power);
            update();
        }
//        t = new ElapsedTime();
//        while (t.milliseconds() < 200)
//            motor.setPowerClassic(intakePwr);
        motor.setPower(0);
    }

    public void autoInAsync() {
        update();
        if (!isFull)
            motor.setPowerClassic(-intakePwr);
        else
            motor.setPowerClassic(0);
    }

    public void autoInAsync(double power) {
        update();
        if (!isFull)
            motor.setPowerClassic(-power);
        else
            motor.setPowerClassic(0);
    }

    public void autoOutAsync() {
        update();
        motor.setPowerClassic(intakePwr);
    }

    public void autoOutAsync(double pwr) {
        update();
        motor.setPowerClassic(pwr);
    }

    public void autoOut() {
        update();
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < intakeMaxTime && !opMode.isStopRequested() && isFull) {
            motor.setPower(intakePwr);
            update();
        }
        t = new ElapsedTime();
        while (t.milliseconds() < 200)
            motor.setPowerClassic(intakePwr);
        motor.setPower(0);
    }

    public void update() {
        averageVoltage = 0;
//
        for (int i = 0; i < 10; i++) {
            if (!input.getState())
                averageVoltage++;
            else
                averageVoltage--;
        }

//        if (averageVoltage < 0.45) // for flying fish
//            isFull = true;
//        else
//            isFull = false;
//        if (!input.getState())
//            isFull = true;
//        else
//            isFull = false;
        if (averageVoltage > 0)
            isFull = true;
        else
            isFull = false;
    }

    public boolean isFull() {
        update();
        return isFull;
    }

    public void setPower(double power) {
        motor.setPowerClassic(power);
    }
}
