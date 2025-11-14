package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mathUtil.PolarVector;

public class SwerveModuleTele {
    DcMotorEx motor;
    CRServo servo;
    AnalogInput encoder;
    ElapsedTime dt = new ElapsedTime();
    double rotationKP = 0.35, rotationKI = 0.0, rotationKD = 0.0;
    double error = 0.0, lastError = 0.0, integral = 0.0;
    double servoBasePower = 0.22;
    double motorDirection = 1.0;
    double encoderMaxVoltage = 5.0;
    double encoderMinVoltage = 0.0;
    double encoderOffset = 0.0;

    public void init (HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotorEx.class, name + "Motor");
        servo = hardwareMap.get(CRServo.class, name + "Servo");
        encoder = hardwareMap.get(AnalogInput.class, name + "Encoder");
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setMaxMinVoltage(double maxVoltage, double minVoltage) {
        encoderMaxVoltage = maxVoltage;
        encoderMinVoltage = minVoltage;
    }

    public void setServoBasePower(double basePower) {
        servoBasePower = basePower;
    }

    public void calculateError (PolarVector target) {
        double error1 = shortest(getWheelDirection(), target.getTheta());
        double error2 = shortest(getWheelDirection(), normalize(target.getTheta()) + Math.PI);
        if (Math.abs(error1) <= Math.abs(error2)) {
            error = error1;
            motorDirection = 1.0;
        } else {
            error = error2;
            motorDirection = -1.0;
        }
    }

    public double PIDController () {
        integral += error * dt.seconds();
        double servoPower = -(error * rotationKP + integral * rotationKI + (error - lastError) / dt.seconds() * rotationKD);
        dt.reset();
        lastError = error;
        servoPower += servoPower > 0.1 ? servoBasePower : servoPower < -0.1 ? -servoBasePower : 0;
        return servoPower;
    }

    public void runModule(PolarVector target) {
        calculateError(target);
        servo.setPower(PIDController());
        motor.setPower(motorDirection * target.getRadius());
    }

    public double getWheelDirection() {
        double voltage = Range.clip(encoder.getVoltage(), encoderMinVoltage, encoderMaxVoltage);
        return (((1 - ((voltage - encoderMinVoltage - (encoderOffset - encoderMinVoltage)) / (encoderMaxVoltage - encoderMinVoltage))) * 2 + 3) * Math.PI) % (2 * Math.PI) - Math.PI;
    }

    public void setEncoderOffset(double offset) {
        encoderOffset = offset;
    }

    public double normalize (double angle) {
        double normalized = angle % (Math.PI * 2);
        if (normalized >= Math.PI) {
            normalized -= 2 * Math.PI;
        } else if (normalized < -Math.PI) {
            normalized += 2 * Math.PI;
        }
        return normalized;
    }

    public double shortest (double from, double to) {
        double difference = to - from;
        if (difference > Math.PI) {
            difference -= 2 * Math.PI;
        } else if (difference < -Math.PI) {
            difference += 2 * Math.PI;
        }
        return difference;
    }
}
