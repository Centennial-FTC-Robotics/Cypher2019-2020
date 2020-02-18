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
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public abstract class CypherMethods extends CypherHardware {

    protected final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private final double ticksPerRotation = 383.6;
    private final double wheelDiameter = 3.937;
    private final double ticksPerWheelRotation = ticksPerRotation; //MULTIPLY BY 2 FOR ACTUAL ROBOT hktdzffd
    private final double distanceInWheelRotation = wheelDiameter * Math.PI;
    private final double ticksPerInch = distanceInWheelRotation / ticksPerWheelRotation;
    private final DcMotorEx[] strafeNeg = new DcMotorEx[2];
    private final DcMotorEx[] strafePos = new DcMotorEx[2];
    private final DcMotorEx[] leftMotors = new DcMotorEx[2];
    private final DcMotorEx[] rightMotors = new DcMotorEx[2];
    final DcMotorEx[] vSlides = new DcMotorEx[2];
    private final DcMotorEx[] wheelIntakeMotors = new DcMotorEx[2];
    private final Servo[] foundationServos = new Servo[2];

    //TODO: Re-finetune these and not break
    //slides break thats how they work
    //private final int VSlideMaxRisk = 1600;
    private final int VSlideMaxSafe = 1400;
    private final int VSlideMin = 10;

    protected int dir;

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
        runWithoutEncoders();

    }

    //MOVEMENT

    void manDriveMotors(double forwardPower, double leftPower, double rotate, double factor) {
        //TODO: smooth controls
        /*consider changing cbrt to sqrt; not sure if this will actually improve the controls,
        but it will definitely decrease the values because using a sqrt would yield a larger number,
        and dividing by a larger number yields an overall smaller value.
         */

        /*
        some ideas for magnitude values:
        1.) Option One:
        double magnitude = Math.log(Math.abs(leftPower + forwardPower + rotate);
        2.) Option Two:
         double magnitude = Math.sqrt(leftPower * leftPower + forwardPower * forwardPower + rotate * rotate) / Math.abs(leftPower + forwardPower + rotate);
        3.) Option Three:
         double magnitude = 3 / 1 + Math.exp(-100(Math.abs(forwardPower + forwardPower + rotate)));
        4.) Option Four:
        double magnitude = 3;
        5.) Option Five:
        double magnitude = Math.pow(3, Math.log(Math.abs(x + y + z) / Math.log(2));

        I'll add more for testing maybe
         */
        double magnitude = Math.sqrt(forwardPower * forwardPower + leftPower * leftPower + rotate * rotate);
        //double magnitude = Math.abs(leftPower + forwardPower + rotate); Use this for cbrt, etc
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

    private void turnAbsolute(double targetAngle) {
        double currentAngle;
        int direction;
        double turnRate;
        double minSpeed = 0.09;
        double maxSpeed = 0.7;
        double tolerance = 1.5;
        double error;
        double P = 1d / 1200;

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
            telemetry.addData("current", currentAngle);
            telemetry.update();
            setDriveMotors((turnRate * direction), -(turnRate * direction));
        }
        while (opModeIsActive() && Math.abs(error) > tolerance);
        setDriveMotors(0);
    }

    void turnRelative(double target) {
        //turnAbsolute(AngleUnit.normalizeDegrees(getRotationDimension('Z') + target));
        turnAbsolute(AngleUnit.normalizeDegrees(getRotationDimension() + target));
    }

    //cause diagonal strafe no work we just move forward then to the side

    protected void testAutoMove(double forward, double left) {
        if (left < forward) {
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

        double P = 1d / 1333;
        double I = 0;
        double tolerance = 15;
        double minSpeed = 0.01;
        double maxSpeed = 0.2333333333333333333333333333;
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
            if (shouldStop())
                stopEverything();
            currentNegPos = getNegPos();
            currentPosPos = getPosPos();

            negError = negTarget - currentNegPos;
            posError = posTarget - currentPosPos;

            negSum += negError;
            posSum += posError;

            negSpeed = clip(P * negError + I * negSum, minSpeed, maxSpeed);
            posSpeed = clip(P * posError + I * posSum, minSpeed, maxSpeed);


            setStrafeMotors(negSpeed, posSpeed);

            telemetry.addData("neg current", currentNegPos);
            telemetry.addData("pos current", currentPosPos);
            telemetry.addData("neg error", negError);
            telemetry.addData("pos error", posError);
            telemetry.addData("neg speed", negSpeed);
            telemetry.addData("pos speed", posSpeed);
            telemetry.addData("forward", forwardMovement);
            telemetry.addData("left", leftMovement);
            telemetry.update();

        } while (opModeIsActive() && (Math.abs(negError) > tolerance || Math.abs(posError) > tolerance));
        setDriveMotors(0);
        setCacheMode(LynxModule.BulkCachingMode.AUTO);
    }

    //TODO: DRIVER ENHANCEMNT SELF CORRECTING STRAFE: do test
    void selfCorrectStrafe(double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        resetEncoders();
        double P = 1 / 1333d;
        double I = 0;
        double tolerance = 5;
        double angleTolerance = 10;
        double minSpeed = 0.01;
        double maxSpeed = 0.5;
        double negSpeed, posSpeed;
        double currentNegPos, currentPosPos;
        double currentAngle;
        double negError, posError;
        double negSum = 0, posSum = 0;
        double angleError;
        double startAngle = getRotationDimension();
        //double startAngle = getRotationDimension('Z');

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        do {
            if (shouldStop())
                stopEverything();
            //currentAngle = getRotationDimension('Z');
            currentAngle = getRotationDimension();
            angleError = currentAngle - startAngle;

            if (Math.abs(angleError) > angleTolerance) {
                turnAbsolute(startAngle);
            }
            if (shouldStop())
                stopEverything();

            currentNegPos = getNegPos();
            currentPosPos = getPosPos();

            negError = currentNegPos - negTarget;
            posError = currentPosPos - posTarget;

            negSum += negError;
            posSum += posError;
            negSpeed = clip(P * negError + I * negSum, minSpeed, maxSpeed);
            posSpeed = clip(P * posError + I * posSum, minSpeed, maxSpeed);

            setStrafeMotors(negSpeed, posSpeed);

            telemetry.addData("neg current", currentNegPos);
            telemetry.addData("pos current", currentPosPos);
            telemetry.addData("neg error", negError);
            telemetry.addData("pos error", posError);
            telemetry.addData("neg speed", negSpeed);
            telemetry.addData("pos speed", posSpeed);
            telemetry.addData("forward", forwardMovement);
            telemetry.addData("left", leftMovement);
            telemetry.update();
        } while (opModeIsActive() && (Math.abs(negError) > tolerance || Math.abs(posError) > tolerance));
        setDriveMotors(0);
    }

    //TODO: Maybe test this one instead>
    protected void betterSelfCorrectStrafe(double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        double P = 1d / 1333;
        double turnP = 1d / 2000;
        double tolerance = 1d / 3;
        double turnTolerance = 2d / 3;
        double minSpeed = 0.01;
        double minTurnSpeed = minSpeed;
        double maxSpeed = 0.5;
        double maxTurnSpeed = 0.3;
        double negError, posError;
        double negPos, posPos;
        double angleError;
        double startAngle = getRotationDimension();
        double currentAngle;
        double negSpeed, posSpeed, rotateSpeed;

        do {
            if (shouldStop())
                stopEverything();
            currentAngle = getRotationDimension();
            negPos = getNegPos();
            posPos = getPosPos();

            negError = negPos - negTarget;
            posError = posPos - posTarget;
            angleError = currentAngle - startAngle;

            negSpeed = clip(P * negError, minSpeed, maxSpeed);
            posSpeed = clip(P * posError, minSpeed, maxSpeed);
            rotateSpeed = clip(angleError * turnP, minTurnSpeed, maxTurnSpeed);
            turnStrafe(negSpeed, posSpeed, rotateSpeed);
        } while (opModeIsActive() && (Math.abs(negError) > tolerance || Math.abs(posError) > tolerance || Math.abs(angleError) > turnTolerance));
        setDriveMotors(0);
    }

    private void turnStrafe(double neg, double pos, double rotate) {
        leftUp.setPower(neg - rotate);
        rightUp.setPower(pos + rotate);
        leftDown.setPower(pos - rotate);
        rightDown.setPower(neg + rotate);
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
        for(DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    void resetEncoders() {
        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        leftDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        for (DcMotorEx motor : vSlides) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
         */
    }


    //INITIALIZE STUFF
    private void initializeIMU() {
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
            while (!imu.isGyroCalibrated() && !isStopRequested());
            resetOrientation();
        } else {
            stopEverything();
        }
    }

    protected void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        if (!isStopRequested())
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        else
            stopEverything();

    }

    protected void initTfod() {
        if(!isStopRequested()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.8;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        } else {
            stopEverything();
        }
    }

    protected void initEverything() {
        initializeIMU();
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }
        telemetry.addLine("Init done");
        telemetry.update();
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
    private double getRotationDimension() {
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
        return rightUp.getCurrentPosition();
    }

    int getVSlidePos() {
        int average = 0;
        for (DcMotorEx motor : vSlides) {
            average += motor.getCurrentPosition();
        }
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
    void controlSlides(double power, double factorThingyKillMeNow) {
        power = clip(power, 0, .8);
        if ((getVSlidePos() >= VSlideMaxSafe && power > 0) || (getVSlidePos() <= VSlideMin && power < 0)) {
            vLeft.setPower(0);
            vRight.setPower(0);
        } else {
            if(power > 0) {
                vLeft.setPower(power);
                vRight.setPower(power * factorThingyKillMeNow);
            } else {
                vLeft.setPower(power);
                vRight.setPower(power);
            }
        }
    }

    void controlSlides(double power) {
        power = clip(power, 0, .8);
        if ((getVSlidePos() >= VSlideMaxSafe && power > 0) || (getVSlidePos() <= VSlideMin && power < 0)) {
            vLeft.setPower(0);
            vRight.setPower(0);
        } else {
            if(power > 0) {
                vLeft.setPower(power);
                vRight.setPower(power * (1d/5));
            } else {
                vLeft.setPower(power);
                vRight.setPower(power* 1.2);
            }
        }
    }

    void moveSlides(int factor) {
        final int moveBy = 150 * factor;
        int a = getVSlidePos() + moveBy;
        if (a >= VSlideMaxSafe) {
            moveSlide(VSlideMaxSafe);
        } else if (a <= VSlideMin) {
            moveSlide(VSlideMin);
        } else {
            moveSlide(moveBy);
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

    protected void waitMili(double mili) {
        ElapsedTime time = new ElapsedTime();
        while (time.milliseconds() < mili && opModeIsActive()) {
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

