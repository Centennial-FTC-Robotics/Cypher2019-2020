package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class CypherMethods extends CypherHardware {
    DcMotor[] driveMotors = new DcMotor[4];

    DcMotor[] leftMotors = new DcMotor[2];
    DcMotor[] rightMotors = new DcMotor[2];

    DcMotor[] strafeNeg = new DcMotor[2];
    DcMotor[] strafePos = new DcMotor[2];

    CRServo[] intakeServos = new CRServo[2];


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

        intakeServos[0] = leftServo;
        intakeServos[1] = rightServo;


    }

    public void autoMove(double forward, double left, double power) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        leftUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftUp.setTargetPosition(forwardMovement - leftMovement);
        rightUp.setTargetPosition(forwardMovement + leftMovement);
        leftDown.setTargetPosition(forwardMovement + leftMovement);
        rightDown.setTargetPosition(forwardMovement - leftMovement);

        setMotorPower(power);

        waitForMotors();

        setMotorPower(0);
    }


    /*                      SIMPLE STUFF TO CHECK MOTOR STATUS                                    */

    public void waitForMotors() {
        while(areMotorsBusy() && opModeIsActive()) {

        }
    }

    public boolean areMotorsBusy() {
        return leftDown.isBusy() || leftUp.isBusy() || rightUp.isBusy() || rightDown.isBusy();
    }
    /*                         THINGS FOR TELEOP I GUESS                                          */
    public void manDriveMotors(double forwardPower, double leftPower, double factor) {
        double magnitude = Math.sqrt(forwardPower * forwardPower + leftPower*leftPower);
        if (magnitude > 1) {
           driveMotors[0].setPower(((-leftPower + forwardPower) / magnitude) * factor);
           driveMotors[1].setPower(((forwardPower + leftPower) / magnitude) * factor);
           driveMotors[2].setPower(((forwardPower + leftPower) / magnitude) * factor);
           driveMotors[3].setPower(((-leftPower + forwardPower) / magnitude) * factor);

        } else {
          driveMotors[0].setPower((-leftPower + forwardPower ) * factor);
          driveMotors[1].setPower((forwardPower + leftPower ) * factor);
          driveMotors[2].setPower((forwardPower + leftPower) * factor);
          driveMotors[3].setPower((-leftPower + forwardPower) * factor);

        }
    }


    public void manRotate(double rotate, double factor) {
        leftMotors[0].setPower(-rotate * factor);
        leftMotors[1].setPower(-rotate * factor);
        rightMotors[0].setPower(rotate * factor);
        rightMotors[1].setPower(rotate * factor);
    }

    /*                     THINGS FOR AUTONONOUS I GUESS                                          */
    public int convertInchToEncoder(double inches) {
        double ticksPerRotation = 383.6;
        double wheelDiameter = 3.937;
        double ticksPerWheelRotation = ticksPerRotation; //MULTIPLY BY 2 FOR ACTUAL ROBOT hktdzffd
        double distanceInWheelRotation = wheelDiameter * Math.PI;
        double ticksPerInch = distanceInWheelRotation/ticksPerWheelRotation;

        double encoderValue = inches/ticksPerInch;
        int intEncoderValue = (int) encoderValue;
        return intEncoderValue;


    }


    public void setMotorPower(double power) {
        for(DcMotor motor: driveMotors) {
            motor.setPower(power);
        }
    }

    public void turnAbsolute() {
        //make th is at some point soon
    }

    public void turnRelative() {
        //make this at some point soon
    }

    public void controlIntakeServos(double power) {
        intakeServos[0].setPower(power);
        intakeServos[1].setPower(power);
    }


}

