package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "Encoder Test", group = "Test")
public class betterEncoder extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetEncoders();
        try {
            initEverything();
        } catch (StopException e) {
            stopEverything();
        }
        waitForStart();
        try {
            skystoneFindPls(-1);
        } catch (StopException e) {
            stopEverything();
        }
    }

    private void skystoneFindPls(int factor) throws StopException {
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
                                    testAutoMove(0, 3 * factor);
                                }
                            } else {
                                telemetry.addData("not skystone", true);
                                testAutoMove(6, 0);
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



