package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CypherMethods extends CypherHardware {
    DcMotor[] motors = new DcMotor[4];
    DcMotor[] leftMotors = new DcMotor[2];
    DcMotor[] rightMotors = new DcMotor[2];

    CRServo[] intakeServos = new CRServo[2];


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        motors[0] = leftUp;
        motors[1] = rightUp;
        motors[2] = leftDown;
        motors[3] = rightDown;

        leftMotors[0] = leftUp;
        leftMotors[1] = leftDown;
        rightMotors[0] = rightUp;
        rightMotors[1] = rightDown;

        intakeServos[0] = leftServo;
        intakeServos[1] = rightMotors;


    }

    public void driveMotors(double fowardPower, double leftPower, double factor) {
        double magnitude = Math.max(Math.abs(leftPower + fowardPower), Math.abs(leftPower - fowardPower));
        if (magnitude > 1) {
            motors[0].setPower(((fowardPower + leftPower) / magnitude) * factor);
            motors[1].setPower(((-fowardPower + leftPower) / magnitude) * factor);
            motors[2].setPower(((-fowardPower + leftPower) / magnitude) * factor);
            motors[3].setPower(((fowardPower + leftPower) / magnitude) * factor);
        } else {
            motors[0].setPower((fowardPower + leftPower) * factor);
            motors[1].setPower((-fowardPower + leftPower) * factor);
            motors[2].setPower((-fowardPower + leftPower) * factor);
            motors[3].setPower((fowardPower + leftPower) * factor);
        }


    }

    public void rotateMotors(double rotate, double factor) {
        leftMotors[0].setPower(rotate * factor);
        leftMotors[1].setPower(rotate * factor);
        rightMotors[0].setPower(-rotate * factor);
        rightMotors[1].setPower(-rotate * factor);
    }

    public void controlIntake(power) {
        

    }

}

