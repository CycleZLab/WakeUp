package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PIN_POINT;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mathUtil.PolarVector;

@Configurable
public class SwerveDrivetrainTele {
    SwerveModuleTele lf = new SwerveModuleTele();
    SwerveModuleTele lr = new SwerveModuleTele();
    SwerveModuleTele rf = new SwerveModuleTele();
    SwerveModuleTele rr = new SwerveModuleTele();
    public GoBildaPinpointDriver pp;
    double theta, power, turn, realTheta;
    public double currentHeading = 0, heading = 0;
    PolarVector lfResult, lrResult, rfResult, rrResult;
    double y, x, rx;

    public void init(HardwareMap hardwareMap) {
        lf.init(hardwareMap, "lf");
        lr.init(hardwareMap, "lr");
        rf.init(hardwareMap, "rf");
        rr.init(hardwareMap, "rr");
        pp = hardwareMap.get(GoBildaPinpointDriver.class, PIN_POINT);
        lf.setMaxMinVoltage(2.980, 0.335);
        lr.setMaxMinVoltage(3.251, 0.006);
        rf.setMaxMinVoltage(3.246, 0.003);
        rr.setMaxMinVoltage(3.252, 0.006);
        lf.setEncoderOffset(2.466);
        lr.setEncoderOffset(3.242);
        rf.setEncoderOffset(0.036);
        rr.setEncoderOffset(0.616);
        pp.setOffsets(0, -47.0, DistanceUnit.MM);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pp.resetPosAndIMU();
        pp.setPosition(new Pose2D(DistanceUnit.INCH, 103.325, 55.325, AngleUnit.DEGREES, 0));
    }

    public void drive(Gamepad gamepad) {
        pp.update();
        currentHeading = (pp.getHeading(AngleUnit.RADIANS) + Math.PI) % (Math.PI * 2) - Math.PI;
        y = -gamepad.left_stick_y;
        x = gamepad.left_stick_x;
        rx = gamepad.right_stick_x * 0.8;
        theta = Math.atan2(x, y);
        power = Math.hypot(x, y);
        turn = rx + 0.001;
        realTheta = ((Math.PI * 2 + currentHeading) + theta - Math.PI) % (Math.PI * 2) - Math.PI;

//        headingError = heading - currentHeading;
//        correctionKP = baseKP * (power * 0.9 + 0.7);
//        correctionKD = baseKD * (power * 0.9 + 0.7);
//        if (headingError > Math.PI)
//            headingError -= 2 * Math.PI;
//        else if (heading - currentHeading < -Math.PI)
//            headingError += 2 * Math.PI;
//        headingCorrection = headingError * correctionKP + (headingError - lastHeadingError) / dt.seconds() * correctionKD;
//        dt.reset();
//        lastHeadingError = headingError;

//        if (Math.abs(rx) > 0) {
//            headingReset.reset();
//        }
//        if (headingReset.milliseconds() < 200) {
//            heading = currentHeading;
//            headingError = 0;
//            lastHeadingError = 0;
//            headingCorrection = 0;
//        }

//        lf.runModule(new PolarVector(power, realTheta).add(new PolarVector(turn + headingCorrection, Math.PI / 4.0)));
//        lr.runModule(new PolarVector(power, realTheta).add(new PolarVector(turn + headingCorrection, -Math.PI / 4.0)));
//        rf.runModule(new PolarVector(power, realTheta).add(new PolarVector(turn + headingCorrection, Math.PI / 4.0 * 3.0)));
//        rr.runModule(new PolarVector(power, realTheta).add(new PolarVector(turn + headingCorrection, -Math.PI / 4.0 * 3.0)));

        lfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0));
        lrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0));
        rfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0 * 3.0));
        rrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0 * 3.0));

        lf.runModule(lfResult);
        lr.runModule(lrResult);
        rf.runModule(rfResult);
        rr.runModule(rrResult);
    }

    public String getWheelDirection() {
        return "\nlf: " + Math.toDegrees(lf.getWheelDirection()) + "\nlr: " + Math.toDegrees(lr.getWheelDirection()) + "\nrf: " + Math.toDegrees(rf.getWheelDirection()) + "\nrr: " + Math.toDegrees(rr.getWheelDirection());
    }

    public String getWheelResults() {
        return "\nlf: " + lfResult.toString() + "\nlr: " + lrResult.toString() + "\nrf: " + rfResult.toString() + "\nrr: " + rrResult.toString();
    }

    public Pose2D getPosition() {
        return pp.getPosition();
    }

    public void setTeleOpDrive(double forward, double strafe, double turn) {
        double currentHeading = 0.0;
        double theta = Math.atan2(strafe, forward);
        double power = Math.hypot(strafe, forward);
        double realTheta = ((Math.PI * 2 + currentHeading) + theta - Math.PI) % (Math.PI * 2) - Math.PI;

        lfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0));
        lrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0));
        rfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0 * 3.0));
        rrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0 * 3.0));

        lf.runModule(lfResult);
        lr.runModule(lrResult);
        rf.runModule(rfResult);
        rr.runModule(rrResult);
    }
}
