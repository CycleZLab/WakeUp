package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@TeleOp
public class VoltageTest extends LinearOpMode {
    AnalogInput lfe, lre, rfe, rre;
    CRServo lfr, lrr, rfr, rrr;
    double lfv, lrv, rfv, rrv;
    double servoPower = 0;
    double lfMax = -1.0, lfMin = 10.0, lrMax = -1.0, lrMin = 10.0, rfMax = -1.0, rfMin = 10.0, rrMax = -1.0, rrMin = 10.0;

    @Override
    public void runOpMode() throws InterruptedException {
        lfe = hardwareMap.get(AnalogInput.class, "lfEncoder");
        lre = hardwareMap.get(AnalogInput.class, "lrEncoder");
        rfe = hardwareMap.get(AnalogInput.class, "rfEncoder");
        rre = hardwareMap.get(AnalogInput.class, "rrEncoder");
        lfr = hardwareMap.get(CRServo.class, "lfServo");
        lrr = hardwareMap.get(CRServo.class, "lrServo");
        rfr = hardwareMap.get(CRServo.class, "rfServo");
        rrr = hardwareMap.get(CRServo.class, "rrServo");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) servoPower += 0.05;
            if (gamepad1.yWasPressed()) servoPower -= 0.05;
            lfr.setPower(servoPower);
            lrr.setPower(servoPower);
            rfr.setPower(servoPower);
            rrr.setPower(servoPower);

            lfv = lfe.getVoltage();
            lrv = lre.getVoltage();
            rfv = rfe.getVoltage();
            rrv = rre.getVoltage();

            if (lfv > lfMax) lfMax = lfv;
            if (lfv < lfMin) lfMin = lfv;
            if (lrv > lrMax) lrMax = lrv;
            if (lrv < lrMin) lrMin = lrv;
            if (rfv > rfMax) rfMax = rfv;
            if (rfv < rfMin) rfMin = rfv;
            if (rrv > rrMax) rrMax = rrv;
            if (rrv < rrMin) rrMin = rrv;

            telemetry.addData("power", servoPower);
            telemetry.addData("lfv", lfv);
            telemetry.addData("lfMax", lfMax);
            telemetry.addData("lfMin", lfMin);
            telemetry.addData("lrv", lrv);
            telemetry.addData("lrMax", lrMax);
            telemetry.addData("lrMin", lrMin);
            telemetry.addData("rfv", rfv);
            telemetry.addData("rfMax", rfMax);
            telemetry.addData("rfMin", rfMin);
            telemetry.addData("rrv", rrv);
            telemetry.addData("rrMax", rrMax);
            telemetry.addData("rrMin", rrMin);
            telemetry.update();
        }
    }
}
