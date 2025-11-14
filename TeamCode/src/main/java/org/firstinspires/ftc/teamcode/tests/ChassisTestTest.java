package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp
public class ChassisTestTest extends LinearOpMode {
    DcMotorEx lf, lr, rf, rr;
    CRServo lfr, lrr, rfr, rrr;
    AnalogInput lfe, lre, rfe, rre;
    double lfeOffset = 0, lreOffset = 0, rfeOffset = 0, rreOffset = 0;
    double lfd, lrd, rfd, rrd;
    double lfTarget, lrTarget, rfTarget, rrTarget;
    double lfTarget2, lrTarget2, rfTarget2, rrTarget2;
    double[] error = new double[4], previousError = new double[4], integral = new double[4], servoPower = new double[4];
    ElapsedTime[] dt = {new ElapsedTime(), new ElapsedTime(), new ElapsedTime(), new ElapsedTime()};
    double rotationKP = 0.0075, rotationKI = 0.0, rotationKD = 0.0;
    double theta, power, turn, realTheta;
    double y, x, rx;
    int lfPowerDir = 1, lrPowerDir = 1, rfPowerDir = 1, rrPowerDir = 1;
    GoBildaPinpointDriver pp;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.get(DcMotorEx.class, "lfMotor");
        lr = hardwareMap.get(DcMotorEx.class, "lrMotor");
        rf = hardwareMap.get(DcMotorEx.class, "rfMotor");
        rr = hardwareMap.get(DcMotorEx.class, "rrMotor");
        lfr = hardwareMap.get(CRServo.class, "lfServo");
        lrr = hardwareMap.get(CRServo.class, "lrServo");
        rfr = hardwareMap.get(CRServo.class, "rfServo");
        rrr = hardwareMap.get(CRServo.class, "rrServo");
        lfe = hardwareMap.get(AnalogInput.class, "lfEncoder");
        lre = hardwareMap.get(AnalogInput.class, "lrEncoder");
        rfe = hardwareMap.get(AnalogInput.class, "rfEncoder");
        rre = hardwareMap.get(AnalogInput.class, "rrEncoder");
        pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");

        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pp.resetPosAndIMU();
        lfeOffset = lfe.getVoltage();
        lreOffset = lre.getVoltage();
        rfeOffset = rfe.getVoltage();
        rreOffset = rre.getVoltage();

        waitForStart();
        while (opModeIsActive()) {
            pp.update();
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            theta = Math.atan2(x, y) * 180 / Math.PI;
            // if (gamepad1.dpadUpWasPressed()) theta = 0;
            // if (gamepad1.dpadDownWasPressed()) theta = 180;
            // if (gamepad1.dpadLeftWasPressed()) theta = -90;
            // if (gamepad1.dpadRightWasPressed()) theta = 90;
            power = Math.hypot(x, y);
            turn = rx;

            realTheta = ((360 + pp.getHeading(AngleUnit.DEGREES)) + theta - 180) % 360 - 180;
            lfd = ((1 - (lfe.getVoltage() - 2.46 - 0.335) / (2.98 - 0.335)) * 360 + 900) % 360 - 180;
            lrd = ((1 - (lre.getVoltage() - lreOffset) / 3.3) * 360 + 540) % 360 - 180;
            rfd = ((1 - (rfe.getVoltage() - rfeOffset) / 3.3) * 360 + 540) % 360 - 180;
            rrd = ((1 - (rre.getVoltage() - rreOffset) / 3.3) * 360 + 540) % 360 - 180;
//            lfTarget = turn > 0.2 ? 45 : 0 + Math.abs(realTheta - lfd) > 90 ? (realTheta + 360) % 180 : realTheta;
//            lrTarget = turn > 0.2 ? -45 : 0 +  Math.abs(realTheta - lrd) > 90 ? (realTheta + 360) % 180 : realTheta;
//            rfTarget = turn > 0.2 ? -45 : 0 +  Math.abs(realTheta - rfd) > 90 ? (realTheta + 360) % 180 : realTheta;
//            rrTarget = turn > 0.2 ? 45 : 0 +  Math.abs(realTheta - rrd) > 90 ? (realTheta + 360) % 180 : realTheta;

            lfTarget = shortest(lfd, realTheta);
            lfTarget2 = shortest(lfd, normalize(realTheta) + 180.0);
            if (Math.abs(lfTarget) <= Math.abs(lfTarget2)) {
                error[0] = lfTarget;
                lfPowerDir = 1;
            } else {
                error[0] = lfTarget2;
                lfPowerDir = -1;
            }
            lf.setPower(power * lfPowerDir);

            lrTarget = shortest(lrd, realTheta);
            lrTarget2 = shortest(lrd, normalize(realTheta) + 180.0);
            if (Math.abs(lrTarget) <= Math.abs(lrTarget2)) {
                error[1] = lrTarget;
                lrPowerDir = 1;
            } else {
                error[1] = lrTarget2;
                lrPowerDir = -1;
            }
            lr.setPower(power * lrPowerDir);

            rfTarget = shortest(rfd, realTheta);
            rfTarget2 = shortest(rfd, normalize(realTheta) + 180.0);
            if (Math.abs(rfTarget) <= Math.abs(rfTarget2)) {
                error[2] = rfTarget;
                rfPowerDir = 1;
            } else {
                error[2] = rfTarget2;
                rfPowerDir = -1;
            }
            rf.setPower(power * rfPowerDir);

            rrTarget = shortest(rrd, realTheta);
            rrTarget2 = shortest(rrd, normalize(realTheta) + 180.0);
            if (Math.abs(rrTarget) <= Math.abs(rrTarget2)) {
                error[3] = rrTarget;
                rrPowerDir = 1;
            } else {
                error[3] = rrTarget2;
                rrPowerDir = -1;
            }
            rr.setPower(power * rrPowerDir);

//            lfr.setPower((error[0] * -0.006) > 0.1 ? (error[0] * -0.006) + 0.22 : (error[0] * -0.006) < -0.1 ? (error[0] * -0.006) - 0.22 : error[0] * -0.006);
//            lrr.setPower((error[1] * -0.006) > 0.1 ? (error[1] * -0.006) + 0.22 : (error[1] * -0.006) < -0.1 ? (error[1] * -0.006) - 0.22 : error[1] * -0.006);
//            rfr.setPower((error[2] * -0.006) > 0.1 ? (error[2] * -0.006) + 0.22 : (error[2] * -0.006) < -0.1 ? (error[2] * -0.006) - 0.22 : error[2] * -0.006);
//            rrr.setPower((error[3] * -0.006) > 0.1 ? (error[3] * -0.006) + 0.22 : (error[3] * -0.006) < -0.1 ? (error[3] * -0.006) - 0.22 : error[3] * -0.006);
            telemetry.addData("lfv", lfe.getVoltage());
            telemetry.addData("lrv", lre.getVoltage());
            telemetry.addData("rfv", rfe.getVoltage());
            telemetry.addData("rrv", rre.getVoltage());
            telemetry.addData("lfd", lfd);
            telemetry.addData("lrd", lrd);
            telemetry.addData("rfd", rfd);
            telemetry.addData("rrd", rrd);
            telemetry.addData("wheelRotationTarget", lfTarget);
            telemetry.addData("wheelRotationError", lfTarget - lfd);
            telemetry.addData("heading", pp.getHeading(AngleUnit.DEGREES));
            telemetry.addData("theta", theta);
            telemetry.addData("realTheta", realTheta);
            for (int i = 0; i < 4; i++) {
                telemetry.addData("error" + i, error[i]);
            }
            for (int i = 0; i < 4; i++) {
                telemetry.addData("i" + i, integral[i]);
            }
            telemetry.update();
        }
    }

    public double PIDController(int index) {
        integral[index] += error[index] * dt[index].seconds();
        servoPower[index] = -(error[index] * rotationKP + integral[index] * rotationKI + (error[index] - previousError[index]) / dt[index].seconds() * rotationKD);
        dt[index].reset();
        previousError[index] = error[index];
        return servoPower[index];
    }

    public double normalize (double angle) {
        double normalized = angle % 360.0;
        if (normalized >= 180.0) {
            normalized -= 360.0;
        } else if (normalized < -180.0) {
            normalized += 360.0;
        }
        return normalized;
    }

    private double shortest (double from, double to) {
        double difference = to - from;
        if (difference > 180.0) {
            difference -= 360.0;
        } else if (difference < -180.0) {
            difference += 360.0;
        }
        return difference;
    }
}