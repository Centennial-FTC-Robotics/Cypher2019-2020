package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TestAutoPIDCalibrator extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetEncoders();

        waitForStart();
        double P = 1 / 1333d;
        double I = 0;
        double D = 0;
        boolean calibrated = false;
        boolean pCalibrated = false;
        boolean iCalibrated = false;
        boolean dCalibrated = false;
        double time;

        while (opModeIsActive()) {
            while (!calibrated) {
                time = runPID(P, I, D, 10, 0);
                if (time > 3) {
                    pCalibrator();
                    runPID(P, I, D, 10, 0);
                    //calibrate P
                } else {
                    dCalibrator();
                    runPID(P, I, D, -10, 0);
                    //calibrate D
                    if (time > 3) {
                        //calibrate I
                    } else {
                        //calibrate D again
                    }
                }
                calibrated = pCalibrated && iCalibrated && dCalibrated;
            }
            telemetry.addData("PID", " calibrated");
            telemetry.update();

        }

    }

    void pCalibrator() {

    }

    void iCalibrator() {

    }

    void dCalibrator() {

    }

    double runPID(double P, double I, double D, double forward, double left) {
        ElapsedTime time = new ElapsedTime();
        PIDCalibrate(P, I, D, 10, 0);
        return time.seconds();
    }

    void PIDCalibrate(double P, double I, double D, double forward, double left) {
        if (Math.abs(left) < Math.abs(forward)) {
            calibrateWithPID(P, I, D, 0, left);
            calibrateWithPID(P, I, D, forward, 0);
        } else {
            calibrateWithPID(P, I, D, forward, 0);
            calibrateWithPID(P, I, D, 0, left);
        }
    }

    void calibrateWithPID(double P, double I, double D, double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        resetEncoders();

        double tolerance = convertInchToEncoder(1);
        double minSpeed = 0.03;
        double maxSpeed = 0.7;
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

}

