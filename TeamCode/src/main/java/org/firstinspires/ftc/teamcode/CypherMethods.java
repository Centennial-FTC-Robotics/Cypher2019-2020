package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class CypherMethods extends CypherHardware {

    private final double ticksPerRotation = 383.6;
    private final double wheelDiameter = 3.937;
    private final double ticksPerWheelRotation = ticksPerRotation; //MULTIPLY BY 2 FOR ACTUAL ROBOT hktdzffd
    private final double distanceInWheelRotation = wheelDiameter * Math.PI;
    private final double ticksPerInch = distanceInWheelRotation / ticksPerWheelRotation;

    final DcMotor[] driveMotors = new DcMotor[4];
    final DcMotor[] strafeNeg = new DcMotor[2];
    final DcMotor[] strafePos = new DcMotor[2];
    private final DcMotor[] leftMotors = new DcMotor[2];
    private final DcMotor[] rightMotors = new DcMotor[2];
    private final DcMotor[] vSlides = new DcMotor[2];
    private final CRServo[] wheelIntakeServos = new CRServo[2];
    private final CRServo[] foundationServos = new CRServo[2];

    private final int VSlideMax = 740;
    private final int VSlideMin = 10;


    int dir;

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

    //MOVEMENT
    private void turnAbsolute(double targetAngle) {
        double currentAngle;
        int direction;
        double turnRate;
        double minSpeed = 0.03;
        double maxSpeed = 0.6;
        double tolerance = 5;
        double error;
        double P = 1d/1200;

        do {
            currentAngle = getRotationinDimension('Z');
            error = getAngleDist(currentAngle, targetAngle);
            direction = getAngleDir(currentAngle, targetAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);
            telemetry.addData("error", error);
            telemetry.addData("turnRate", turnRate);
            telemetry.addData("current", currentAngle);
            telemetry.update();
            setDriveMotors((turnRate * direction), -(turnRate * direction));
        }
        while (opModeIsActive() && error > tolerance);
        setDriveMotors(0);
    }

    void testAutoMove(double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        resetEncoders();

        double P = 1d / 1200;
        double I = 0;
        double tolerance = 1d/3;
        double minSpeed = 0.01;
        double maxSpeed = 0.5;
        double negSpeed, posSpeed;
        double currentNegPos, currentPosPos;
        double negError, posError;
        double negSum = 0, posSum = 0;

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        do {
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

    void selfCorrectStrafe(double forward, double left) {
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
        double startAngle = getRotationinDimension('Z');

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        do {
            currentAngle = getRotationinDimension('Z');
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

    void setDriveMotors(double leftPower, double rightPower) {
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

    void turnRelative(double target) {
        turnAbsolute(AngleUnit.normalizeDegrees(getRotationinDimension('Z') + target));
    }

    //INITIALIZE STUFF
    void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        while (opModeIsActive() && !imu.isGyroCalibrated()) ;
        resetOrientation();

    }

    //METHODS THAT ASSIST WITH AUTONOMOUS IDK
    public double getRotationinDimension(char dimension) {

        switch (Character.toUpperCase(dimension)) {
            case 'X':

                return AngleUnit.normalizeDegrees(rawDimension('X') - initialPitch);
            case 'Y':

                return AngleUnit.normalizeDegrees(rawDimension('Y') - initialRoll);
            case 'Z':
                return AngleUnit.normalizeDegrees(rawDimension('Z') - initialHeading);
        }

        return 0;
    }

    private double rawDimension(char dimension) {
        orientationUpdate();
        switch (dimension) {
            case 'Z':
                return orientation.firstAngle;
            case 'Y':
                return orientation.thirdAngle;
            case 'X':
                return orientation.secondAngle;
        }
        return 0;
    }

    double getAngleDist(double targetAngle, double currentAngle) {
        double angleDifference = currentAngle - targetAngle;

        if (Math.abs(angleDifference) > 180) {
            angleDifference = 360 - Math.abs(angleDifference);
        } else {
            angleDifference = Math.abs(angleDifference);
        }

        return angleDifference;
    }


    int getAngleDir(double targetAngle, double currentAngle) {
        double angleDifference = targetAngle - currentAngle;
        int angleDir = (int) (angleDifference / Math.abs(angleDifference));

        if (Math.abs(angleDifference) > 180) {
            angleDir *= -1;
        }

        return angleDir;
    }

    int getNegPos() {
        int average = 0;
        for (DcMotor motor : strafeNeg) {
            average += motor.getCurrentPosition();
        }
        return average / 2;
    }

    int getPosPos() {
        /*int average = 0;
        for (DcMotor motor : strafePos) {
            average += motor.getCurrentPosition();
        }
        return average / 2;
          */
        return rightUp.getCurrentPosition();
    }

    int getVSlidePos() {
        int average = 0;
        for (DcMotor motor : vSlides) {
            average += motor.getCurrentPosition();
        }
        return average / 2;
    }

    int getPos() {
        return (getNegPos() + getPosPos()) / 2;
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

    private void orientationUpdate() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private void resetOrientation() {
        orientationUpdate();
        initialHeading = orientation.firstAngle;
        initialRoll = orientation.secondAngle;
        initialPitch = orientation.thirdAngle;
    }


    //CONVERSION METHODS
    int convertInchToEncoder(double inches) {
        return (int) (inches / ticksPerInch);
    }

    int convertEncoderToInch(int encoder) {
        return (int) (ticksPerInch / encoder);
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
    //BIll was here

    void grabServo(int pos) {
        arm.setPosition(pos);
    }

    void swivelServo(double power) {

        swivel.setPower(power);
    }

    void controlSlides(double power) {
        for (DcMotor motor : vSlides) {
            if ((getVSlidePos() >= VSlideMax && power < 0) || (getVSlidePos() <= VSlideMin && power > 0)) {
                motor.setPower(0);
            } else {
                motor.setPower(clip(power, 0, .2));
            }
        }

    }

    void moveFoundation(double power) {
        for (CRServo servo : foundationServos) {
            servo.setPower(power);
        }
    }


    void controlFoundation(FoundationState state) {
        ElapsedTime time = new ElapsedTime();
        if (state.equals(FoundationState.RELEASE)) {
            time.reset();
            moveFoundation(-1);
            while (time.milliseconds() < 650) ;
            moveFoundation(0);
            time.reset();
        } else {
            time.reset();
            moveFoundation(1);
            while (time.milliseconds() < 350) ;
            moveFoundation(0.3);
            time.reset();
        }
    }

    double acutalControl(double controller, double a) {
        //a*b^3+(1-a)*b
        return (a * (Math.pow(-controller, 3))) + ((1 - a) * -controller);
    }

    double getArmPos() {
        return arm.getPosition();
    }

    double clip(double num, double min, double max) {
        int sign;
        if (num < 0) {
            sign = -1;
        } else {
            sign = 1;
        }
        if (Math.abs(num) < min) return min * sign;
        if (Math.abs(num) > max) return max * sign;

        return num;
    }

    void skystoneFindPls(int factor) {
        final double tolerance = 200;
        resetEncoders();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                setDriveMotors(0);
                                telemetry.addData("SKYSTONE", true);
                                telemetry.addData("left", recognition.getLeft());
                                telemetry.addData("right", recognition.getRight());
                                if (Math.abs(recognition.getRight() - recognition.getLeft()) > tolerance) {
                                    moveToCenter(recognition.getLeft(), recognition.getRight());
                                    telemetry.addData("moving", "to skystone.........");
                                } else {
                                    telemetry.addData("moving", "to the side.........");
                                }
                            } else {
                                telemetry.addData("not skystone", true);
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

    void moveToCenter(double left, double right) {
        double P = 0.02;
        double error = left - right;
        double speed;
        double minSpeed = 0.01;
        double maxSpeed = 0.03;
        speed = clip(P * error, minSpeed, maxSpeed);

        setDriveMotors(speed);
    }

    void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    void initEverything() {
        initializeIMU();
        initVuforia();
        initTfod();


        if (tfod != null) {
            tfod.activate();
        }
    }

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

