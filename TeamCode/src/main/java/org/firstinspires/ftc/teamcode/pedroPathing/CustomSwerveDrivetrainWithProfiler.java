package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mathUtil.PolarVector;

import java.util.Arrays;
import java.util.List;

public class CustomSwerveDrivetrainWithProfiler extends Drivetrain {
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx leftRearMotor;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx rightRearMotor;
    private final CRServo leftFrontServo;
    private final CRServo leftRearServo;
    private final CRServo rightFrontServo;
    private final CRServo rightRearServo;
    private final AnalogInput leftFrontEncoder;
    private final AnalogInput leftRearEncoder;
    private final AnalogInput rightFrontEncoder;
    private final AnalogInput rightRearEncoder;
    private final List<DcMotorEx> motors;
    private final List<CRServo> servos;
    private final List<AnalogInput> encoders;
    private final VoltageSensor voltageSensor;
    private List<Double> encoderMaxVoltage;
    private List<Double> encoderMinVoltage;
    private List<Double> encoderOffset;
    private List<Double> servoBasePower;
    private List<Double> rotationKP;
    private List<Double> rotationKI;
    private List<Double> rotationKD;
    private double[] motorDirection = {1.0, 1.0, 1.0, 1.0};
    private double[] error = {0.0, 0.0, 0.0, 0.0};
    private double[] lastError = {0.0, 0.0, 0.0, 0.0};
    private double[] integral = {0.0, 0.0, 0.0, 0.0};
    private ElapsedTime[] dt = {new ElapsedTime(), new ElapsedTime(), new ElapsedTime(), new ElapsedTime()};
    private double motorCachingThreshold;
    private double staticFrictionCoefficient;
    public double[] debugArr = new double[6], truePathing = new double[4];
    public PolarVector leftSide, rightSide;

    public CustomSwerveConstants constants;

    public CustomSwerveDrivetrainWithProfiler(HardwareMap hardwareMap, CustomSwerveConstants swerveConstants) {
        constants = swerveConstants;

        this.motorCachingThreshold = swerveConstants.motorCachingThreshold;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, swerveConstants.leftFrontMotorName);
        leftRearMotor = hardwareMap.get(DcMotorEx.class, swerveConstants.leftRearMotorName);
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, swerveConstants.rightFrontMotorName);
        rightRearMotor = hardwareMap.get(DcMotorEx.class, swerveConstants.rightRearMotorName);
        leftFrontServo = hardwareMap.get(CRServo.class, swerveConstants.leftFrontServoName);
        leftRearServo = hardwareMap.get(CRServo.class, swerveConstants.leftRearServoName);
        rightFrontServo = hardwareMap.get(CRServo.class, swerveConstants.rightFrontServoName);
        rightRearServo = hardwareMap.get(CRServo.class, swerveConstants.rightRearServoName);
        leftFrontEncoder = hardwareMap.get(AnalogInput.class, swerveConstants.leftFrontEncoderName);
        leftRearEncoder = hardwareMap.get(AnalogInput.class, swerveConstants.leftRearEncoderName);
        rightFrontEncoder = hardwareMap.get(AnalogInput.class, swerveConstants.rightFrontEncoderName);
        rightRearEncoder = hardwareMap.get(AnalogInput.class, swerveConstants.rightRearEncoderName);

        motors = Arrays.asList(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
        servos = Arrays.asList(leftFrontServo, leftRearServo, rightFrontServo, rightRearServo);
        encoders = Arrays.asList(leftFrontEncoder, leftRearEncoder, rightFrontEncoder, rightRearEncoder);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
        breakFollowing();

        updateConstants();

        Vector copiedFrontLeftVector = swerveConstants.frontLeftVector.normalize();
        vectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())
        };
    }

    public void updateConstants() {
        leftFrontMotor.setDirection(constants.leftFrontMotorDirection);
        leftRearMotor.setDirection(constants.leftRearMotorDirection);
        rightFrontMotor.setDirection(constants.rightFrontMotorDirection);
        rightRearMotor.setDirection(constants.rightRearMotorDirection);
        encoderMaxVoltage = Arrays.asList(constants.lfEncoderMaxVoltage,
                constants.lrEncoderMaxVoltage,
                constants.rfEncoderMaxVoltage,
                constants.rrEncoderMaxVoltage);
        encoderMinVoltage = Arrays.asList(constants.lfEncoderMinVoltage,
                constants.lrEncoderMinVoltage,
                constants.rfEncoderMinVoltage,
                constants.rrEncoderMinVoltage);
        encoderOffset = Arrays.asList(constants.lfEncoderOffset,
                constants.lrEncoderOffset,
                constants.rfEncoderOffset,
                constants.rrEncoderOffset);
        servoBasePower = Arrays.asList(constants.lfServoBasePower,
                constants.lrServoBasePower,
                constants.rfServoBasePower,
                constants.rrServoBasePower);
        rotationKP = Arrays.asList(constants.lfRotationKP,
                constants.lrRotationKP,
                constants.rfRotationKP,
                constants.rrRotationKP);
        rotationKI = Arrays.asList(constants.lfRotationKI,
                constants.lrRotationKI,
                constants.rfRotationKI,
                constants.rrRotationKI);
        rotationKD = Arrays.asList(constants.lfRotationKD,
                constants.lrRotationKD,
                constants.rfRotationKD,
                constants.rrRotationKD);
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    public void breakFollowing() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        setMotorsToFloat();
    }

    @Override
    public void startTeleopDrive() {
        setMotorsToBrake();
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        setMotorsToBrake();
    }

    public double xVelocity() {
        return constants.xVelocity;
    }

    public double yVelocity() {
        return constants.yVelocity;
    }

    public void setXVelocity(double xMovement) {
        constants.setXVelocity(xMovement);
    }

    public void setYVelocity(double yMovement) {
        constants.setYVelocity(yMovement);
    }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) /
                (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    @Override
    public String debugString() {
        return "\ncorrective: " + String.format("%.2f", debugArr[0]) + ", " + String.format("%.2f", debugArr[1]) +
                "\nheading: " + String.format("%.2f", debugArr[2]) + ", " + String.format("%.2f", debugArr[3]) +
                "\npathing: " + String.format("%.2f", debugArr[4]) + ", " + String.format("%.2f", debugArr[5]) +
                "\ntp0: " + String.format("%.2f", truePathing[0]) + ", " + String.format("%.2f", truePathing[1]) +
                "\ntp1: " + String.format("%.2f", truePathing[2]) + ", " + String.format("%.2f", truePathing[3]) +
                "\nleft: " + (leftSide == null ? "null" : leftSide.toString()) +
                "\nright: " + (rightSide == null ? "null" : rightSide.toString());
    }

    public double getWheelDirection(int index) {
        double voltage = Range.clip(encoders.get(index).getVoltage(), encoderMinVoltage.get(index), encoderMaxVoltage.get(index));
        return (((1 - ((voltage - encoderMinVoltage.get(index) - (encoderOffset.get(index) - encoderMinVoltage.get(index))) /
                (encoderMaxVoltage.get(index) - encoderMinVoltage.get(index)))) * 2 + 3) * Math.PI) % (2 * Math.PI) - Math.PI;
    }

    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public double normalize(double angle) {
        double normalized = angle % (Math.PI * 2);
        if (normalized >= Math.PI) {
            normalized -= 2 * Math.PI;
        } else if (normalized < -Math.PI) {
            normalized += 2 * Math.PI;
        }
        return normalized;
    }

    public double shortest(double from, double to) {
        double difference = to - from;
        if (difference > Math.PI) {
            difference -= 2 * Math.PI;
        } else if (difference < -Math.PI) {
            difference += 2 * Math.PI;
        }
        return difference;
    }

    public void calculateError(PolarVector target, int index) {
        double error1 = shortest(getWheelDirection(index), target.getTheta());
        double error2 = shortest(getWheelDirection(index), normalize(target.getTheta()) + Math.PI);
        if (Math.abs(error1) <= Math.abs(error2)) {
            error[index] = error1;
            motorDirection[index] = 1.0;
        } else {
            error[index] = error2;
            motorDirection[index] = -1.0;
        }
    }

    public double PIDController(int index) {
        integral[index] += error[index] * dt[index].seconds();
        double servoPower = -(error[index] * rotationKP.get(index) +
                integral[index] * rotationKI.get(index) +
                (error[index] - lastError[index]) / dt[index].seconds() * rotationKD.get(index));
        dt[index].reset();
        lastError[index] = error[index];
        servoPower += servoPower > 0.1 ? servoBasePower.get(index) :
                servoPower < -0.1 ? -servoBasePower.get(index) : 0;
        return servoPower;
    }

    public void runModule(PolarVector target, int index) {
        calculateError(target, index);
        servos.get(index).setPower(PIDController(index));
        motors.get(index).setPower(motorDirection[index] * target.getRadius());
    }
    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        if (correctivePower.getMagnitude() >= maxPowerScaling) {
            correctivePower.setMagnitude(maxPowerScaling);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    0
            };
        }
        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        if (scaleDown(correctivePower, headingPower, true)) {
            headingPower = scaledVector(correctivePower, headingPower, true);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    headingPower.dot(new Vector(-1, robotHeading))
            };
        } else {
            Vector combinedStatic = correctivePower.plus(headingPower);
            if (scaleDown(combinedStatic, pathingPower, false)) {
                pathingPower = scaledVector(combinedStatic, pathingPower, false);
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(-1, robotHeading))
                };
            } else {
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(-1, robotHeading))
                };
            }
        }
    }
//    @Override
//    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
//        double max = maxPowerScaling;
//
//        if (correctivePower.getMagnitude() >= max) {
//            correctivePower.setMagnitude(max);
//            return new double[] {
//                    correctivePower.getXComponent(),
//                    correctivePower.getYComponent(),
//                    0
//            };
//        }
//
//        if (headingPower.getMagnitude() > max) headingPower.setMagnitude(max);
//        if (pathingPower.getMagnitude() > max) pathingPower.setMagnitude(max);
//        double cMag = correctivePower.getMagnitude();
//        double hMag = headingPower.getMagnitude();
//        double pMag = pathingPower.getMagnitude();
//
//        double cNorm = (max > 1e-6) ? cMag / max : 0.0;
//        double hNorm = (max > 1e-6) ? hMag / max : 0.0;
//
//        double a = 0.7;
//        double b = 0.7;
//        double sPath = clamp(1.0 - a * cNorm - b * hNorm, 0.0, 1.0);
//
//        if (cMag > 1e-4 && pMag > 1e-4 && sPath > 0.0) {
//            double cosPhi = correctivePower.dot(pathingPower) / (cMag * pMag);
//            if (cosPhi < 0.0) {
//                double angleFactor = 1.0 + cosPhi;
//                if (angleFactor < 0.0) angleFactor = 0.0;
//                sPath *= angleFactor;
//            }
//        }
//
//        Vector translationalField = correctivePower.plus(pathingPower.times(sPath));
//        double transMag = translationalField.getMagnitude();
//
//        double turn = headingPower.dot(new Vector(-1.0, robotHeading));
//        double turnMag = Math.abs(turn);
//
//        if (transMag > max) {
//            translationalField.setMagnitude(max);
//            transMag = max;
//        }
//        if (turnMag > max) {
//            turn = Math.signum(turn) * max;
//            turnMag = max;
//        }
//        double sumSq = Math.pow(transMag, 2) + Math.pow(turnMag, 2);
//
//        if (sumSq > 1.0 + 1e-6) {
//            double scale = 1.0 / Math.sqrt(sumSq);
//            translationalField.setMagnitude(transMag * scale);
//            turn *= scale;
//        }
//
//        return new double[] {
//                translationalField.getXComponent(),
//                translationalField.getYComponent(),
//                turn
//        };
//    }



    private boolean scaleDown(Vector staticVector, Vector variableVector, boolean useMinus) {
        return (staticVector.plus(variableVector).getMagnitude() >= maxPowerScaling) ||
                (useMinus && staticVector.minus(variableVector).getMagnitude() >= maxPowerScaling);
    }

    private Vector scaledVector(Vector staticVector, Vector variableVector, boolean useMinus) {
        double scalingFactor = useMinus
                ? Math.min(
                findNormalizingScaling(staticVector, variableVector, maxPowerScaling),
                findNormalizingScaling(staticVector, variableVector.times(-1), maxPowerScaling)
        )
                : findNormalizingScaling(staticVector, variableVector, maxPowerScaling);
        return variableVector.times(scalingFactor);
    }

    @Override
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        double[] calculatedDrive = calculateDrive(correctivePower, headingPower, pathingPower, robotHeading);

        Vector translationalVector = new Vector();
        translationalVector.setOrthogonalComponents(calculatedDrive[0], calculatedDrive[1]);

        lastPathingPower = pathingPower;
        lastCorrectivePower = correctivePower;
        lastTranslationalVector = translationalVector;
        lastHeadingPower = headingPower;
        lastHeading = robotHeading;

        translationalVector.rotateVector(-robotHeading);
        arcadeDrive(translationalVector.getXComponent(),
                translationalVector.getYComponent(),
                calculatedDrive[2]);
    }

    @Deprecated
    @Override
    public void runDrive(double[] drivePowers) {
    }
    public void arcadeDrive(double forward, double strafe, double rotation) {
        strafe *= -1;
        double theta = Math.atan2(strafe, forward);
        double power = Math.hypot(strafe, forward);
        double realTheta = ((Math.PI * 2) + theta - Math.PI) % (Math.PI * 2) - Math.PI;

        PolarVector lfResult = new PolarVector(power, realTheta).add(new PolarVector(rotation, Math.PI / 4.0));
        PolarVector lrResult = new PolarVector(power, realTheta).add(new PolarVector(rotation, -Math.PI / 4.0));
        PolarVector rfResult = new PolarVector(power, realTheta).add(new PolarVector(rotation, 3.0 * Math.PI / 4.0));
        PolarVector rrResult = new PolarVector(power, realTheta).add(new PolarVector(rotation, -3.0 * Math.PI / 4.0));

        runModule(lfResult, 0);
        runModule(lrResult, 1);
        runModule(rfResult, 2);
        runModule(rrResult, 3);
    }

    public void setTeleOpDrive(double forward, double strafe, double turn) {
        arcadeDrive(forward, strafe, turn);
    }

    protected Vector lastTranslationalVector = new Vector();
    protected Vector lastHeadingPower = new Vector();
    protected Vector lastCorrectivePower = new Vector();
    protected Vector lastPathingPower = new Vector();
    protected double lastHeading = 0;
}
