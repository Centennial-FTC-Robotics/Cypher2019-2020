package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


public abstract class CypherMethods extends CypherHardware {

    protected static double TILE_LENGTH = 22.75;
    protected final DcMotorEx[] driveMotors = new DcMotorEx[4];
    final DcMotorEx[] vSlides = new DcMotorEx[2];
    private final double ticksPerRotation = 383.6;
    private final double wheelDiameter = 3.937;
    private final double ticksPerWheelRotation = ticksPerRotation; //MULTIPLY BY 2 FOR ACTUAL ROBOT hktdzffd
    private final double distanceInWheelRotation = wheelDiameter * Math.PI;
    private final double ticksPerInch = distanceInWheelRotation / ticksPerWheelRotation;
    private final DcMotorEx[] strafeNeg = new DcMotorEx[2];
    private final DcMotorEx[] strafePos = new DcMotorEx[2];
    private final DcMotorEx[] leftMotors = new DcMotorEx[2];
    private final DcMotorEx[] rightMotors = new DcMotorEx[2];
    private final DcMotorEx[] wheelIntakeMotors = new DcMotorEx[2];
    private final Servo[] foundationServos = new Servo[2];
    //will set these tmrw (tues)
    private final int vSlideMax = 1000000;
    private final int vSlideMin = -100000;
    protected int dir;

    IntakeState intakeState = IntakeState.STOP;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        driveMotors[0] = leftUp;
        driveMotors[1] = rightUp;
        driveMotors[2] = leftDown;
        driveMotors[3] = rightDown;

        leftMotors[0] = leftUp;
        leftMotors[1] = leftDown;
        rightMotors[0] = rightUp;
        rightMotors[1] = rightDown;

        strafeNeg[0] = leftUp;
        strafeNeg[1] = rightDown;

        strafePos[0] = rightUp;
        strafePos[1] = leftDown;

        wheelIntakeMotors[0] = leftIntake;
        wheelIntakeMotors[1] = rightIntake;

        vSlides[0] = vLeft;
        vSlides[1] = vRight;

        foundationServos[0] = lFoundation;
        foundationServos[1] = rFoundation;
        resetEncoders();
        setCacheMode(LynxModule.BulkCachingMode.OFF);
        //runWithoutEncoders();

    }

    //MOVEMENT

    void manDriveMotors(double forwardPower, double leftPower, double rotate, double factor) {
        double magnitude = Math.sqrt(forwardPower * forwardPower + leftPower * leftPower + rotate * rotate);
        if (magnitude > 1) {
            strafeNeg[0].setPower(((-leftPower + forwardPower - rotate) / magnitude) * factor);
            strafePos[0].setPower(((forwardPower + leftPower + rotate) / magnitude) * factor);
            strafePos[1].setPower(((forwardPower + leftPower - rotate) / magnitude) * factor);
            strafeNeg[1].setPower(((-leftPower + leftPower + rotate) / magnitude) * factor);
        } else {
            strafeNeg[0].setPower((-leftPower + forwardPower - rotate) * factor);
            strafePos[0].setPower((forwardPower + leftPower + rotate) * factor);
            strafePos[1].setPower((forwardPower + leftPower - rotate) * factor);
            strafeNeg[1].setPower((-leftPower + forwardPower + rotate) * factor);
        }
    }

    void turnAbsolute(double targetAngle) {
        double currentAngle;
        int direction;
        double turnRate;
        double minSpeed = 0.1;
        double maxSpeed = 0.5;
        double tolerance = 1;
        double error;
        double P = 1d / 1400;
        int count = 0;

        do {
            if (shouldStop())
                stopEverything();
            //currentAngle = getRotationDimension('Z');
            currentAngle = getRotationDimension();
            error = getAngleDist(currentAngle, targetAngle);
            direction = getAngleDir(currentAngle, targetAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);
            telemetry.addData("error", error);
            telemetry.addData("turnRate", turnRate);
            telemetry.addData("count", count);
            telemetry.addData("current", currentAngle);
            telemetry.update();
            setDriveMotors((turnRate * direction), -(turnRate * direction));
            if(Math.abs(error) > tolerance) {
                count++;
            } else if(Math.abs(error) <= tolerance) {
                count = 0;
            }

        }
        while (opModeIsActive() && Math.abs(error) > tolerance && count < 10);
        setDriveMotors(0);
    }

    void turnRelative(double target) {
        //turnAbsolute(AngleUnit.normalizeDegrees(getRotationDimension('Z') + target));
        turnAbsolute(AngleUnit.normalizeDegrees(getRotationDimension() + target));
    }

    //cause diagonal strafe no work we just move forward then to the side

    protected void testAutoMove(double forward, double left) {
        if (Math.abs(left) < Math.abs(forward)) {
            actualMove(0, left);
            actualMove(forward, 0);
        } else {
            actualMove(forward, 0);
            actualMove(0, left);
        }
    }

    private void actualMove(double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        resetEncoders();

        double P = 1d / 1222;
        double I = 0;
        double tolerance = 6;
        double minSpeed = 0.03;
        double maxSpeed = 0.4;
        double negSpeed, posSpeed;
        double currentNegPos, currentPosPos;
        double negError, posError;
        double negSum = 0, posSum = 0;

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;
        setCacheMode(LynxModule.BulkCachingMode.MANUAL);
        do {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            currentNegPos = getNegPos();
            currentPosPos = getPosPos();

            negError = negTarget - currentNegPos;
            posError = posTarget - currentPosPos;

            negSum += negError;
            posSum += posError;

            negSpeed = clip(P * negError + I * negSum, minSpeed, maxSpeed);
            posSpeed = clip(P * posError + I * posSum, minSpeed, maxSpeed);


            setStrafeMotors(negSpeed, posSpeed);
            telemetry.addLine("auto move telemetry");
            telemetry.addData("left down power", leftDown.getPower());
            telemetry.addData("neg current", currentNegPos);
            telemetry.addData("pos current", currentPosPos);
            telemetry.addData("neg error", negError);
            telemetry.addData("pos error", posError);
            telemetry.update();

            if (shouldStop()) {
                stopEverything();
                break;
            }
        } while (opModeIsActive() && (Math.abs(negError) > tolerance || Math.abs(posError) > tolerance));
        setDriveMotors(0);
        setCacheMode(LynxModule.BulkCachingMode.AUTO);
    }

    void turnStrafe(double neg, double pos, double rotate) {
        leftUp.setPower(neg + rotate);
        rightUp.setPower(pos - rotate);
        leftDown.setPower(pos + rotate);
        rightDown.setPower(neg - rotate);
    }


    protected void stopEverything() {
        telemetry.addLine("Stopping");
        telemetry.update();
        for (DcMotorEx motor : driveMotors) {
            motor.setPower(0);
        }
        for (DcMotorEx motor : vSlides) {
            motor.setPower(0);
        }
        for (DcMotorEx motor : wheelIntakeMotors) {
            motor.setPower(0);
        }
        HSlide.setPower(0);

    }

    //MOTOR CONTROL
    private void setDriveMotors(double leftPower, double rightPower) {
        for (DcMotorEx motor : leftMotors) {
            motor.setPower(leftPower);
        }
        for (DcMotorEx motor : rightMotors) {
            motor.setPower(rightPower);
        }
    }


    void setDriveMotors(double power) {
        for (DcMotorEx motor : driveMotors) {
            motor.setPower(power);
        }
    }

    void setStrafeMotors(double neg, double pos) {
        for (DcMotorEx motor : strafeNeg) {
            motor.setPower(neg);
        }
        for (DcMotorEx motor : strafePos) {
            motor.setPower(pos);
        }
    }

    void runWithoutEncoders() {
        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    void resetEncoders() {
        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        //leftDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        for (DcMotorEx motor : vSlides) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
         */
    }

    void resetVSlideEncoder() {
        for (DcMotorEx motor : vSlides) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }


    //INITIALIZE STUFF
    protected void initializeIMU() {
        if (!isStopRequested()) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(parameters);
        } else {
            stopEverything();
        }
        while (!imu.isGyroCalibrated() && !isStopRequested()) ;
        resetOrientation();

    }

    private void orientationUpdate() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private void resetOrientation() {
        orientationUpdate();
        initialHeading = orientation.firstAngle;
        initialRoll = orientation.secondAngle;
        initialPitch = orientation.thirdAngle;
    }

    //GETTING STUFF LIKE MOTOR POSITION, ORIENTATION, AND STUFF LIKE THAT IDK
    double getRotationDimension() {
        return AngleUnit.normalizeDegrees(rawDimension() - initialHeading);
    }

    private double rawDimension() {
        orientationUpdate();
        return orientation.firstAngle;
    }

    private double getAngleDist(double targetAngle, double currentAngle) {
        double angleDifference = currentAngle - targetAngle;

        if (Math.abs(angleDifference) > 180) {
            angleDifference = 360 - Math.abs(angleDifference);
        } else {
            angleDifference = Math.abs(angleDifference);
        }

        return angleDifference;
    }


    private int getAngleDir(double targetAngle, double currentAngle) {
        double angleDifference = targetAngle - currentAngle;
        int angleDir = (int) (angleDifference / Math.abs(angleDifference));

        if (Math.abs(angleDifference) > 180) {
            angleDir *= -1;
        }

        return angleDir;
    }

    int getNegPos() {
        int average = 0;
        for (DcMotorEx motor : strafeNeg) {
            average += motor.getCurrentPosition();
        }
        return average / 2;
    }

    int getPosPos() {
        int average = 0;
        for (DcMotorEx motor : strafePos) {
            average += motor.getCurrentPosition();
        }
        //return rightUp.getCurrentPosition(); //should be the average of rightUp and backLeft but encoder for backLeft no work so this is the best we can do
        return average / 2;
    }


    int getPos() {
        return (getNegPos() + getPosPos()) / 2;
    }

    //CONVERSION METHODS
    protected int convertInchToEncoder(double inches) {
        return (int) (inches / ticksPerInch);
    }

    double convertEncoderToInch(int encoder) {
        return ticksPerInch / encoder;
    }

    double tilesToInch(double tiles) {
        return tiles * 22.75;
    }

    double convertInchToTile(double tiles) {
        return tiles / 24;
    }

    //INTAKE METHODS
    void controlIntakeMotors(double power) {
        wheelIntakeMotors[0].setPower(power);
        wheelIntakeMotors[1].setPower(power);

        if(power > 0)
            intakeState = IntakeState.IN;
        else if(power < 0)
            intakeState = IntakeState.OUT;
        else
            intakeState = IntakeState.STOP;
    }

    void controlArm(double power) {
        HSlide.setPower(power);
    }

    void grabServo(double pos) {
        arm.setPosition(pos);
    }

    //ARM & SLIDES
    private void moveSlide(int pos) {
        for (DcMotorEx motor : vSlides) {
            motor.setTargetPosition(pos);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPower(.3);
        }
    }


    void controlSlides(double power, int oldEncoder) {
        int encoderDiff = 0;
        power = clip(power, 0, .8);
        if ((vSlideEncoder >= vSlideMax && power > 0) || (vRight.getCurrentPosition() <= vSlideMin && power < 0)) {
            vLeft.setPower(0);
            vRight.setPower(0);
        } else {
            if (power > 0)
                encoderDiff = oldEncoder - vLeft.getCurrentPosition();
            else if (power < 0)
                encoderDiff = oldEncoder - vRight.getCurrentPosition();
            telemetry.addData("diff", encoderDiff);
            vSlideEncoder += encoderDiff;

            if (power > 0) {
                vLeft.setPower(power);
                vRight.setPower(power * (1d / 3));
            } else {
                vLeft.setPower(power * .8);
                vRight.setPower(power);
            }
        }
    }


    //FOUNDATION THINGY
    void moveFoundation(double pos) {
        for (Servo servo : foundationServos) {
            servo.setPosition(pos);
        }
    }

    void controlFoundation(FoundationState state) {
        if (state.equals(FoundationState.RELEASE)) {
            moveFoundation(1);
        } else {
            lFoundation.setPosition(0.1);
            rFoundation.setPosition(0.05);
        }

    }

    //mathy math math stuff idk
    double actualControl(double controller, double a) {
        //a*b^3+(1-a)*b
        return (a * (Math.pow(-controller, 3))) + ((1 - a) * -controller);
    }

    protected void waitMilli(double mili) {
        ElapsedTime time = new ElapsedTime();
        while (time.milliseconds() < mili && opModeIsActive()) {
            if (shouldStop())
                stopEverything();
        }
    }

    protected void waitSec(double secs) {
        ElapsedTime time = new ElapsedTime();
        while (time.seconds() < secs && opModeIsActive()) {
            if (shouldStop())
                stopEverything();
        }
    }

    double clip(double num, double min, double max) {
        int sign;
        if (num < 0)
            sign = -1;
        else
            sign = 1;
        if (Math.abs(num) < min)
            return min * sign;
        else if (Math.abs(num) > max)
            return max * sign;
        else
            return num;
    }

    protected void setCacheMode(LynxModule.BulkCachingMode mode) {
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(mode);
        }
    }

    boolean notInitController() {
        return !(gamepad1.start || gamepad2.start);
    }

    boolean shouldStop() {
        return isStopRequested() || (!opModeIsActive());
    }

    void changeColor(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinLed.setPattern(pattern);
    }

    double tuner(double P) {
        //resetEncoders();
        //waitForStart();
        ElapsedTime controllerTimer = new ElapsedTime();
        final int miliTillReady = 250;
        int factor = 1;

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean right = gamepad1.right_bumper;
            boolean left = gamepad1.left_bumper;
            boolean cont = gamepad1.start;

            if (controllerTimer.milliseconds() > miliTillReady) {
                if (a) {
                    P += 0.0005;
                    controllerTimer.reset();
                }
                if (y) {
                    P += 0.0001;
                    controllerTimer.reset();

                }
                if (right) {
                    P += 0.005;
                    controllerTimer.reset();

                }
                if (b) {
                    P -= 0.0005;
                    controllerTimer.reset();

                }
                if (x) {
                    P -= 0.0001;
                    controllerTimer.reset();

                }
                if (left) {
                    P -= 0.001;
                    controllerTimer.reset();

                }
                if (cont) {
                    factor *= -1;
                    controllerTimer.reset();

                }
            }
            telemetry.addData("P", P);
            telemetry.update();
        }
        return P;
    }

    //enum stuff
    enum IntakeState {
        IN, OUT, STOP
    }

    enum FoundationState {
        DRAG, RELEASE
    }

    enum ArmState {
        PICK, DROP, REST
    }


}

