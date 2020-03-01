package org.firstinspires.ftc.teamcode;

public class UnusedMethods {
    /* DElete after season is oveer along with badauto
    private double getRotationDimension(char dimension) {
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
    double getArmPos() {
        return arm.getPosition();
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
    //Tile
    int getInX() {
        return innerTile.x;
    }

    int getInY() {
        return innerTile.y;
    }

    private double[] getDist(Tile start, Tile end, int dir) {
        double forward, left;
        /* //Note: Code to apply for any given angle. Not sure if it works, so it's commented. Let's tests.
        if (dir >= 0) {
            directionNew = 360 - (dir - 90);
        }
        else {
            directionNew = -dir + 90;
        }
        int startHorizontalAdjust = start.getX()*Math.cos(directionNew) + start.getY()*Math.sin(directionNew);
        int startVerticalAdjust = -start.getX()*Math.sin(directionNew) + start.getY()*Math.cos(directionNew);
        int finalHorizontalAdjust = end.getX()*Math.cos(directionNew) + end.getY()*Math.sin(directionNew);
        int finalVerticalAdjust = -end.getX()*Math.sin(directionNew) + end.getY()*Math.cos(directionNew);
        forward = finalVerticalAdjust - startVerticalAdjust;
        left = finalHorizontalAdjust - startHorizontalAdjust;
        return new double[]{tilesToInch(forward), tilesToInch(left)};
        }
private void moveToCenter(double left, double right) {
        double P = 0.02;
        double error =  right - left;
        double speed;
        double minSpeed = 0.01;
        double maxSpeed = 0.03;
        speed = Range.clip(P * error, minSpeed, maxSpeed);
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.update();

}
        */
/*someidea I had that isn't needed(park to the right side(AKA non-wall side) of bridge
public class RedBottomMoveFoundationRight extends CypherAutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        try {
            initEverything();
        } catch (StopException e) {
            stopEverything();
        }
        waitForStart();
        getFoundation(-1, Side.RIGHTWALL);


    }
}
 */
/*
    protected void testPIDThingy(double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);
        double kP = 1d / 2222;
        double kI = 1d / 3000;
        double kD = 0;
        double tolerance = 1d / 3;
        double deltaTime, oldTime = 0;
        double minSpeed = 0.03;
        double maxSpeed = 0.5;
        ElapsedTime runtime = new ElapsedTime();

        Value in index 0 is for the neg motors
        Value in index 1 is for the pos motors
        Using arrays for this and not different variables to stop us from having 50 different
        variables and arrays are also just cool

        double[] proportional = new double[2];
        double[] integral = new double[2];
        double[] derivative = new double[2];
        double[] speed = new double[2];
        double[] pos = new double[2];
        double[] error = new double[2];
        double[] oldError = {0, 0};
        double[] target = {forwardMovement - leftMovement, forwardMovement + leftMovement};
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean right = gamepad1.right_bumper;
        boolean lef = gamepad1.left_bumper;
        boolean cont = gamepad1.start;
        ElapsedTime controllerTimer = new ElapsedTime();
        setCacheMode(LynxModule.BulkCachingMode.MANUAL);
        resetEncoders();
        do {
            if (a) {
                kI += 0.0005;
                controllerTimer.reset();
            }
            if (y) {
                kI += 0.0001;
                controllerTimer.reset();

            }
            if (right) {
                kI += 0.005;
                controllerTimer.reset();

            }
            if (b) {
                kI -= 0.0005;
                controllerTimer.reset();

            }
            if (x) {
                kI -= 0.0001;
                controllerTimer.reset();

            }
            if (lef) {
                kI -= 0.001;
                controllerTimer.reset();

            }
            /*if (cont) {
                //findP(0, 22.75 * factor, P); unsure why this is here will delete if not resolved
                factor *= -1;
                controllerTimer.reset();

            }
            telemetry.addData("P", kI);
            telemetry.update();
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
            if (shouldStop())
                stopEverything();
            pos[0] = getNegPos();
            pos[1] = getPosPos();
            deltaTime = Math.abs(runtime.seconds() - oldTime);
            for (int i = 0; i < 2; i++) {
                error[i] = target[i] - pos[i];
                proportional[i] = kP * error[i];
                integral[i] += error[i] * deltaTime;
                derivative[i] = (error[i] - oldError[i]) / deltaTime; //highly sure you mixed up errors or so - idek how it works tbh lol
                speed[i] = clip(proportional[i] + integral[i] * kI + derivative[i] * kD, minSpeed, maxSpeed);
                oldError[i] = error[i];
            }
            setStrafeMotors(speed[0], speed[1]);

            telemetry.addData("neg error", error[0]);
            telemetry.addData("pos error", error[1]);
            telemetry.addData("neg speed", speed[0]);
            telemetry.addData("pos speed", speed[1]);
            telemetry.addData("neg P", proportional[0]);
            telemetry.addData("pos P", proportional[1]);
            telemetry.addData("neg integral", integral[0] * kI);
            telemetry.addData("pos integral", integral[1] * kI);
            telemetry.addData("forward", forwardMovement);
            telemetry.addData("left", leftMovement);
            telemetry.update();
            oldTime = runtime.seconds();

        } while (opModeIsActive() && (Math.abs(error[0]) > tolerance || Math.abs(error[1]) > tolerance));
        setDriveMotors(0);
        setCacheMode(LynxModule.BulkCachingMode.AUTO);
    }
    idk what this even does
    package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Disabled
@Autonomous(name = "Encoder Test", group = "Test")
public class betterEncoder extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetEncoders();
        initEverything();
        waitForStart();

        skystoneFindPls(-1);

    }

    private void skystoneFindPls(int factor) {
        final double tolerance = 200;
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
                                    autoMove(0, 3 * factor);
                                }
                            } else {
                                telemetry.addData("not skystone", true);
                                autoMove(6, 0);
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

    private void moveToCenter(double left, double right) {
        double P = 0.02;
        double error = left - right;
        double minSpeed = 0.01;
        double maxSpeed = 0.03;
        setDriveMotors(clip(P * error, minSpeed, maxSpeed));
    }


}




 */


}
