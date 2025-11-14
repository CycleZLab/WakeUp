package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class DeviceTest extends LinearOpMode {
    private int motorNum = 8, servoNum = 12, touchNum = 0;
    private DcMotorEx[] motors = new DcMotorEx[motorNum];
    private Servo[] servos = new Servo[servoNum];
    private DigitalChannel[] touches = new DigitalChannel[touchNum];
    private int servoIndex = 0, motorIndex = 0;
    private boolean upHold = false, downHold = false, yHold = false, aHold = false;

    @Override
    public void runOpMode() {
        for (int i = 0; i < motorNum; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, ("m" + i));
        }
        for (int i = 0; i < servoNum; i++) {
            servos[i] = hardwareMap.get(Servo.class, ("s" + i));
        }
        for (int i = 0; i < touchNum; i++) {
            touches[i] = hardwareMap.get(DigitalChannel.class, ("t" + i));
            touches[i].setMode(DigitalChannel.Mode.INPUT);
        }

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !upHold) {
                upHold = true;
                servoIndex++;
            } else if (!gamepad1.dpad_up) {
                upHold = false;
            }
            if (gamepad1.dpad_down && !downHold) {
                downHold = true;
                servoIndex--;
            } else if (!gamepad1.dpad_down) {
                downHold = false;
            }
            if (gamepad1.y && !yHold) {
                yHold = true;
                motorIndex++;
            } else if (!gamepad1.y) {
                yHold = false;
            }
            if (gamepad1.a && !aHold) {
                aHold = true;
                motorIndex--;
            } else if (!gamepad1.a) {
                aHold = false;
            }

            if (gamepad1.right_bumper) {
                if (motorIndex >= motorNum) {
                    motorIndex = motorNum - 1;
                } else if (motorIndex < 0) {
                    motorIndex = 0;
                }
                motors[motorIndex].setPower(1);
            } else {
                for (int i = 0; i < motorNum; i++) {
                    motors[i].setPower(0);
                }
            }

            if (gamepad1.left_bumper) {
                if (servoIndex >= servoNum) {
                    servoIndex = servoNum - 1;
                } else if (servoIndex < 0) {
                    servoIndex = 0;
                }
                servos[servoIndex].setPosition(1);
            } else if (gamepad1.left_trigger > 0) {
                if (servoIndex >= servoNum) {
                    servoIndex = servoNum - 1;
                } else if (servoIndex < 0) {
                    servoIndex = 0;
                }
                servos[servoIndex].setPosition(0);
            }

            for (int i = 0; i < touchNum && opModeIsActive(); i++) {
                if (touches[i].getState()) {
                    telemetry.addData("Digital Touch" + i, "Is Not Pressed");
                } else {
                    telemetry.addData("Digital Touch" + i, "Is Pressed");
                }
            }

            telemetry.addData("Servo", "s" + servoIndex);
            telemetry.addData("Motor", "m" + motorIndex);
            telemetry.update();
        }
    }
}
