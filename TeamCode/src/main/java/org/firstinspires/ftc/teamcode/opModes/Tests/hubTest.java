package org.firstinspires.ftc.teamcode.opModes.Tests;

import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.CONTROL_HUB;
import static org.firstinspires.ftc.teamcode.modules.HardwareConfig.EXPANSION_HUB;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.firstinspires.ftc.teamcode.misc.ExpansionHubEx;

@TeleOp
@Disabled
public class hubTest extends LinearOpMode {
//    private ExpansionHubEx expHub, cHub;
    private LynxVoltageSensor controlHubVoltageSensor, expansionHubVoltageSensor;
    private HardwareMap hardwareMap;
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap = OpModeManagerImpl.getOpModeManagerOfActivity(
                AppUtil.getInstance().getRootActivity()).getHardwareMap();
//        cHub = new ExpansionHubEx(hardwareMap.get(LynxModule.class, CONTROL_HUB));
//        expHub = new ExpansionHubEx(hardwareMap.get(LynxModule.class, EXPANSION_HUB));

//        try {
//            controlHubVoltageSensor = new LynxVoltageSensor(hardwareMap.appContext, cHub.getStandardModule());
//        } catch (RobotCoreException e) {
//            e.printStackTrace();
//        }
//        try {
////            expansionHubVoltageSensor = new LynxVoltageSensor(hardwareMap.appContext, expHub.getStandardModule());
//        } catch (RobotCoreException e) {
//            e.printStackTrace();
//        }

//        expHub.setLedColor(255, 0, 255);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("chub voltage", controlHubVoltageSensor.getVoltage());
            telemetry.addData("exphub voltage", expansionHubVoltageSensor.getVoltage());
            telemetry.update();
        }
    }
}
