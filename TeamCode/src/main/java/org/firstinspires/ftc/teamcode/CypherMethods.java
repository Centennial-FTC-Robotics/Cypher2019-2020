package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public abstract class CypherMethods extends CypherHardware {

    private final double ticksPerRotation = 383.6;
    private final double wheelDiameter = 3.937;
    private final double ticksPerWheelRotation = ticksPerRotation; //MULTIPLY BY 2 FOR ACTUAL ROBOT hktdzffd
    private final double distanceInWheelRotation = wheelDiameter * Math.PI;
    private final double ticksPerInch = distanceInWheelRotation / ticksPerWheelRotation;

    private final DcMotor[] driveMotors = new DcMotor[4];
    private final DcMotor[] strafeNeg = new DcMotor[2];
    private final DcMotor[] strafePos = new DcMotor[2];
    private final DcMotor[] leftMotors = new DcMotor[2];
    private final DcMotor[] rightMotors = new DcMotor[2];
    private final DcMotor[] vSlides = new DcMotor[2];
    private final CRServo[] wheelIntakeServos = new CRServo[2];
    private final Servo[] foundationServos = new Servo[2];

    private final int VSlideMax = 760;
    private final int VSlideMin = 5;


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

        wheelIntakeServos[0] = leftServo;
        wheelIntakeServos[1] = rightServo;

        vSlides[0] = vLeft;
        vSlides[1] = vRight;

        foundationServos[0] = lFoundation;
        foundationServos[1] = rFoundation;
        resetEncoders();
    }

    //MOVEMENT
    
    void manDriveMotors(double forwardPower, double leftPower, double rotate, double factor) {
        double magnitude = Math.cbrt(forwardPower * forwardPower + leftPower * leftPower + rotate * rotate);
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

    private void turnAbsolute(double targetAngle) throws StopException {
        double currentAngle;
        int direction;
        double turnRate;
        double minSpeed = 0.09;
        double maxSpeed = 0.7;
        double tolerance = 7;
        double error;
        double P = 1d / 1200;

        do {
            if (shouldStop()) {
                throw new StopException("stap");
            }
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

    void turnRelative(double target) throws StopException {
        //turnAbsolute(AngleUnit.normalizeDegrees(getRotationDimension('Z') + target));
        turnAbsolute(AngleUnit.normalizeDegrees(getRotationDimension() + target));
    }

    //cause diagonal strafe no work we just move forward then to the side
    protected void testAutoMove(double forward, double left) throws StopException {
        if (left < forward) {
            actualMove(0, left);
            actualMove(forward, 0);
        } else {
            actualMove(forward, 0);
            actualMove(0, left);
        }
    }

    private void actualMove(double forward, double left) throws StopException {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        resetEncoders();

        double P = 1d / 1333;
        double I = 0;
        double tolerance = convertInchToEncoder(1d / 3);
        double minSpeed = 0.01;
        double maxSpeed = 0.2666;
        double negSpeed, posSpeed;
        double currentNegPos, currentPosPos;
        double negError, posError;
        double negSum = 0, posSum = 0;

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        do {
            if (shouldStop()) {
                throw new StopException("stap");
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

    //still needs testing
    void selfCorrectStrafe(double forward, double left) throws StopException {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        resetEncoders();
        double P = 0.04;
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
            if (shouldStop()) {
                throw new StopException("stap");
            }
            //currentAngle = getRotationDimension('Z');
            currentAngle = getRotationDimension();
            angleError = currentAngle - startAngle;

            if (Math.abs(angleError) > angleTolerance) {
                turnRelative(angleError);
            }

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

    protected void stopEverything() {
        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
        }
        for (DcMotor motor : vSlides) {
            motor.setPower(0);
        }
        for (CRServo servo : wheelIntakeServos) {
            servo.setPower(0);
        }
        swivel.setPower(0);
        HSlide.setPower(0);
    }

    //MOTOR CONTROL
    private void setDriveMotors(double leftPower, double rightPower) {
        for (DcMotor motor : leftMotors) {
            motor.setPower(leftPower);
        }
        for (DcMotor motor : rightMotors) {
            motor.setPower(rightPower);
        }
    }


    void setDriveMotors(double power) {
        for (DcMotor motor : driveMotors) {
            motor.setPower(power);
        }
    }

    void setStrafeMotors(double neg, double pos) {
        for (DcMotor motor : strafeNeg) {
            motor.setPower(neg);
        }
        for (DcMotor motor : strafePos) {
            motor.setPower(pos);
        }
    }

    void resetEncoders() {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        for (DcMotor motor : vSlides) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    //INITIALIZE STUFF
    private void initializeIMU() throws StopException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {
            if (shouldStop()) {
                throw new StopException("stap");
            }
        }
        resetOrientation();
    }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    protected void initEverything() throws StopException {
        initializeIMU();
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }
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

    private int getNegPos() {
        int average = 0;
        for (DcMotor motor : strafeNeg) {
            average += motor.getCurrentPosition();
        }
        return average / 2;
    }

    private int getPosPos() {
        int average = 0;
        for (DcMotor motor : strafePos) {
            average += motor.getCurrentPosition();
        }
        return average / 2;
    }

    private int getVSlidePos() {
        int average = 0;
        for (DcMotor motor : vSlides) {
            average += motor.getCurrentPosition();
        }
        return average / 2;
    }

    int getPos() {
        return (getNegPos() + getPosPos()) / 2;
    }


    //CONVERSION METHODS
    private int convertInchToEncoder(double inches) {
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
    void controlIntakeServos(double power) {
        wheelIntakeServos[0].setPower(power);
        wheelIntakeServos[1].setPower(power);
    }

    void controlArm(double power) {
        HSlide.setPower(power);
    }

    void grabServo(double pos) {
        arm.setPosition(pos);
    }

    //ARM & SLIDES
    void swivelServo(double power) {
        swivel.setPower(power);
    }

    private void moveSlide(int pos) {
        for (DcMotor motor : vSlides) {
            motor.setTargetPosition(pos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(.3);
        }
    }

    void controlSlides(double power) {
        for (DcMotor motor : vSlides) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if ((getVSlidePos() >= VSlideMax && power < 0) || (getVSlidePos() <= VSlideMin && power > 0)) {
                motor.setPower(0);
            } else {
                motor.setPower(clip(power, 0, .15));
            }
        }
    }

    void moveSlides(int factor) {
        final int moveBy = 150 * factor;
        int a = getVSlidePos() + moveBy;
        if (a >= VSlideMax) {
            moveSlide(VSlideMax);
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
        }
        else {
            moveFoundation(0.05);
        }

    }

    //mathy math math stuff idk
    double actualControl(double controller, double a) {
        //a*b^3+(1-a)*b
        return (a * (Math.pow(-controller, 3))) + ((1 - a) * -controller);
    }


    double clip(double num, double min, double max) {
        int sign;
        if (num < 0) {
            sign = -1;
        } else {
            sign = 1;
        }
        if (Math.abs(num) < min) {
            return min * sign;
        } else if (Math.abs(num) > max) {
            return max * sign;
        } else {
            return num;
        }

    }

    boolean notInitController() {
        return !(gamepad1.start || gamepad2.start);
    }

    boolean shouldStop() {
        return isStopRequested() || !opModeIsActive();
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

