package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Encoder Test", group = "Test")
public class betterEncoder extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetEncoders();
        waitForStart();
        double P = 0.04;
        //Bill was here
        ElapsedTime controllerTimer = new ElapsedTime();
        final int miliTillReady = 250;
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
                    P += 0.01;
                    controllerTimer.reset();
                }
                if (y) {
                    P += 0.1;
                    controllerTimer.reset();

                }
                if (right) {
                    P += 0.5;
                    controllerTimer.reset();

                }
                if (b) {
                    P -= 0.01;
                    controllerTimer.reset();

                }
                if (x) {
                    P -= 0.1;
                    controllerTimer.reset();

                }
                if (left) {
                    P -= 0.5;
                    controllerTimer.reset();

                }
                if (cont) {
                    findP(22.75, 0, P);
                    controllerTimer.reset();

                }
            }
            telemetry.addData("P", P);
            telemetry.update();
        }


    }


    private void findP(double forward, double left, double P) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        resetEncoders();

        double I = 0;
        double tolerance = 5;
        double minSpeed = 0.01;
        double maxSpeed = 0.3;
        double negSpeed, posSpeed;
        double currentNegPos, currentPosPos;
        double negError, posError;
        double negSum = 0, posSum = 0;

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        do {
            currentNegPos = getNegPos();
            currentPosPos = getPosPos();

            negError =   currentNegPos - negTarget;
            posError =  currentPosPos  - posTarget;

            negSum += negError;
            posSum += posError;

            negSpeed = clip(P * negError + I * negSum, minSpeed, maxSpeed);
            posSpeed = clip(P * posError + I * posSum, minSpeed, maxSpeed);

            setStrafeMotors(negSpeed, posSpeed);

            telemetry.addData("right up", rightUp.getPower());
            telemetry.addData("right down", rightDown.getPower());
            telemetry.addData("left up", leftUp.getPower());
            telemetry.addData("left down", leftDown.getPower());

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
}



