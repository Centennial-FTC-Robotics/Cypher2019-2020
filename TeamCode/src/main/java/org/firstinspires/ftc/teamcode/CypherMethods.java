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
    final Tile currentPos = new Tile(0, 0); //always start here
    private final Tile redFoundation = new Tile(5, 5.5);
    private final Tile redBuildSite = new Tile(6, 5.5);
    private final Tile redQuarry = new Tile(5.5, 2);
    private final Tile redBridge = new Tile(5.5, 3.5);
    private final Tile blueFoundation = new Tile(2, 5.5);
    private final Tile blueBuildSite = new Tile(1, 5.5);
    private final Tile blueQuarry = new Tile(1.5, 2);
    private final Tile blueBridge = new Tile(1.5, 3.5);

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
        double P = 0.04;
        double minSpeed = 0.01;
        double maxSpeed = 0.4;
        double tolerance = 5;
        double error;

        do {
            currentAngle = dimensionRotation();
            error = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);
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

        double P = 0.04;
        double I = 0;
        double tolerance = 5;
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
            telemetry.addData("aaaaaaaaaaa", "aaaaaaaaaaaaaaaaa");
            telemetry.addData("egfdg", Range.clip(-1, minSpeed, maxSpeed));
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
        double startAngle = dimensionRotation();

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        do {
            currentAngle = dimensionRotation();
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

    private void setStrafeMotors(double neg, double pos) {
        for (DcMotor motor : strafeNeg) {
            motor.setPower(neg);
        }
        for (DcMotor motor : strafePos) {
            motor.setPower(pos);
        }
    }

    private void turnRelative(double target) {
        turnAbsolute(AngleUnit.normalizeDegrees(dimensionRotation() + target));
    }

    //INITIALIZE STUFF
   void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        while (opModeIsActive() && !imu.isGyroCalibrated()) ;
        resetOrientation();
    }

    //METHODS THAT ASSIST WITH AUTONOMOUS IDK
    private double dimensionRotation() {
        return AngleUnit.normalizeDegrees(rawDimension() - initialHeading);
    }

    private double rawDimension() {
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

    private int getPos() {
        return (getNegPos() + getPosPos()) / 2;
    }

    private void resetEncoders() {
        for (DcMotor motor : driveMotors) {
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
    private int convertInchToEncoder(double inches) {
        return (int)(inches / ticksPerInch);
      }

    private int convertEncoderToInch(int encoder) {
        return (int)(ticksPerInch / encoder);
    }

    private double tilesToInch(double tiles) {
        return tiles * 24;
    }

    private double convertInchToTile(double tiles) {
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

    void grabServo(double power) {
        arm.setPower(power);
    }

    void swivelServo(double power) {

        swivel.setPower(power);
    }

    void controlSlides(double power) {
        for(DcMotor motor : vSlides) {
            motor.setPower(clip(power, 0, .4));
        }

    }

    private void moveFoundation(double power) {
        foundation.setPower(power);
    }


    void controlFoundation(FoundationState state) {
        ElapsedTime time = new ElapsedTime();
        if(state.equals(FoundationState.RELASE)) {
            time.reset();
            moveFoundation(-1);
            while(time.milliseconds() < 650  );
            moveFoundation(0);
            time.reset();
        } else {
            time.reset();
            moveFoundation(1);
            while(time.milliseconds() < 350);
            moveFoundation(0.3);
            time.reset();
        }
    }

    double acutalControl(double controller) {
        double a = 0.3;
        //a*b^3+(1-a)*b
        return (a*(Math.pow(controller, 3))) + ((1-a)* controller);
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
                                telemetry.addData("SKYSTONE", true);
                                telemetry.addData("left", recognition.getLeft());
                                telemetry.addData("right", recognition.getRight());
                                if (Math.abs(recognition.getRight() - recognition.getLeft()) > tolerance) {
                                    moveToCenter(recognition.getLeft(), recognition.getRight());
                                } else {
                                    testAutoMove(0, 6 * factor);
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

    private double[] getDist(Tile start, Tile end, int dir) {
        double forward, left;
        switch (dir) {
            case 90:
                forward = end.getX() - start.getX();
                left = end.getY() - start.getY();
                break;
            case 180:
                forward = start.getY() - end.getY();
                left = end.getX() - start.getX();
                break;
            case -90:
                forward = start.getX() - end.getX();
                left = end.getY() - start.getY();
                return new double[]{tilesToInch(forward), tilesToInch(left)};
            default:
                forward = end.getY() - start.getY();
                left = end.getX() - start.getX();
                break;
        }
        return new double[]{tilesToInch(forward), tilesToInch(left)};
    }

    private void moveToPos(double x2, double y2, int dir) {
        moveToPos(new Tile(x2, y2), dir);
    }

    private void moveToPos(Tile end, int dir) {
        double[] move = getDist(currentPos, end, dir);
        testAutoMove(move[0], move[1]);
        currentPos.setLocation(end);
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
        ElapsedTime timer = new ElapsedTime();
        initializeIMU();
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("time", timer.milliseconds());
        telemetry.update();

    }

    private void waitControlIntake(double power) {
        ElapsedTime time = new ElapsedTime();
        controlIntakeServos(power);
        while (time.milliseconds() < 200) ;
    }

    private void waitMoveFoundation(double power) {
        ElapsedTime time = new ElapsedTime();
        moveFoundation(power);
        while (time.milliseconds() < 200) ;
    }

    //AUTO STUFF
    void buildingAuto(String side) {
        int factor = 1;
        switch (side) {
            case "red":
                factor = 1;
                break;
            case "blue":
                factor = -1;
        }
        Tile oldPos;
        moveToPos(currentPos.getX() - .5 * factor, currentPos.getY(), dir); //move forward a small bit
        turnRelative(180); //turn around so we can pick up foundation
        dir = 90 * factor;
        if (factor == 1)
            moveToPos(redFoundation, dir); //go to foundation
        else
            moveToPos(blueFoundation, dir);
        waitMoveFoundation(1);

        if (factor == 1)
            moveToPos(redBuildSite, dir);
        else
            moveToPos(blueBuildSite, dir);
        waitMoveFoundation(-1);
        moveFoundation(0);
        if (factor == 1)
            moveToPos(redQuarry, dir); //go to red quarry
        else
            moveToPos(blueQuarry, dir);
        turnRelative(-90 * factor);
        dir = 180;

        for (int i = 0; i < 2; i++) { //repeat twice for 2 skystones

            skystoneFindPls(factor); //center with skystone
            currentPos.add(0, -convertInchToTile(convertEncoderToInch(getPos()))); //find how far we travelled to find skystone
            oldPos = new Tile(currentPos);
            moveToPos(currentPos.getX() - convertInchToTile(factor), currentPos.getY(), dir); //move to be right behind/infront/whatever of skystone

            //pick up skystone and move into it
            controlIntakeServos(1);
            testAutoMove(2, 0);
            currentPos.add(0, convertInchToTile(-2));

            moveToPos(currentPos.getX() + factor, currentPos.getY(), dir); //move a bit to prevent hitting the neutral bridge
            moveToPos(currentPos.getX(), currentPos.getY() + 2, dir); //move to other side

            turnRelative(-90 * factor); //turn to release skystone and not have it in the way
            dir = -90 * factor;
            waitControlIntake(-1); //release skystone
            controlIntakeServos(0);
            turnRelative(90 * factor); //turn back
            dir = 180;
            if (i == 0) //if that was the first skystone move back to where we got the first one to look for second ome
                moveToPos(oldPos, dir);
        }
        if (factor == 1)
            moveToPos(redBridge, dir); //go to red bridge
        else
            moveToPos(blueBridge, dir); //or blue bridge
    }

    void loadingAuto(String side) {
        int factor = 1;
        switch (side) {
            case "red":
                factor = 1;
                break;
            case "blue":
                factor = -1;
                break;
        }

        testAutoMove(2, 0);
        currentPos.add(convertInchToTile(2) * factor, 0);
        turnRelative(-90 * factor);
        dir = 180;
        for (int i = 0; i < 2; i++) {
            skystoneFindPls(factor);
            currentPos.add(0, -convertInchToTile(convertEncoderToInch(getPos()))); //find how far we travelled to find skystone
            Tile oldPos = new Tile(currentPos);
            moveToPos(currentPos.getX() - convertInchToTile(factor), currentPos.getY(), dir); //move to be right behind/infront/whatever of skystone

            waitControlIntake(1);
            testAutoMove(2, 0);
            currentPos.add(convertInchToTile(2), 0);

            moveToPos(currentPos.getX() + factor, currentPos.getY(), dir); //move a bit to prevent hitting the neutral bridge
            moveToPos(currentPos.getX(), blueBridge.getY() + 1.5, dir); //move to other side

            turnRelative(-90 * factor); //turn to spit out block w/o it getting in way
            dir = -90 * factor; //change dir
            waitControlIntake(-1); //spit it out

            turnRelative(180);
            dir *= -1;
            if (i == 0) { //if its the first skystone move foundation
                if (factor == 1) {
                    moveToPos(redFoundation, dir);

                } else {
                    moveToPos(blueFoundation, dir);
                }
                waitMoveFoundation(1);
                if (factor == 1) {
                    moveToPos(redBuildSite, dir);
                } else {
                    moveToPos(blueBuildSite, dir);
                }
                waitMoveFoundation(-1);
                moveFoundation(0);
            }

            if(i == 0) {
                moveToPos(oldPos, dir);
            }

            turnRelative(90 * factor);
            dir = 180;
        }
        if (factor == 1) {
            moveToPos(redBridge, dir);
        } else {
            moveToPos(blueBridge, dir);
        }

    }

    enum IntakeState {
        IN, OUT, STOP
    }

    enum FoundationState {
            DRAG, RELASE
    }

    enum ArmState {
        PICK, DROP, REST
    }




}

