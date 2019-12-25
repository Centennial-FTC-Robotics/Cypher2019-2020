package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Tensor Flow Test", group = "Test")
public class tensorFlowTest extends CypherMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initVuforia();
        initTfod();


        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();

        skystoneFindPls(1);

    }

/*
    public void goToSkystone() {
        double tolerance = 200; //close enough value
        while (opModeIsActive()) {
            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update();

                    for (Recognition recognition : updatedRecognitions) {
                        double left = recognition.getLeft();
                        double right = recognition.getRight();
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {  //if skystone is detected
                            if (Math.abs(left - right) > tolerance) { //if its not "close enough"
                                moveToCenter(left, right); //run the thingy that makes it "close enough"
                                telemetry.addData("left", left);
                                telemetry.addData("right", right);
                                telemetry.update();
                            } else {
                                setDriveMotors(0); //when u r close enough stop moving

                            }

                        } else { //if not skystone
                            testAutoMove(0, 12); //move to side
                            telemetry.addData("left", left);
                            telemetry.addData("right", right);
                            telemetry.update();
                        }
                        telemetry.addData("left", left);
                        telemetry.addData("right", right);
                        telemetry.addData("type", recognition.getLabel());
                        telemetry.update();
                    }

                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }

        }
    }
*/

/*
    public void findSkystone() {
        while (opModeIsActive()) {
            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update(); //added to see if its apps fault; it's not(this shows fine but not ones in for loop)
                    int i = 0;

                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {  //if skystone is detected
                            double left = recognition.getLeft();
                            double right = recognition.getRight();

                            if (right == left || Math.abs(left - right) < tolerance) {
                                testAutoMove(12, 0); //negative makes it move forward
                            }

                           else if ((left > right) || (right > left)) {
                                double surpassLevel;
                                double behindLevel;
                                boolean differentiate;
                                int direction;
                                if ((left > right)) {
                                    surpassLevel = left;
                                    behindLevel = right;
                                    differentiate = false;
                                } else {
                                    surpassLevel = right;
                                    behindLevel = left;
                                    differentiate = true;
                                }

                                while (surpassLevel > behindLevel && opModeIsActive() && Math.abs(left - right) > tolerance) {
                                    newLeft = recognition.getLeft();
                                    newRight = recognition.getRight();
                                    if (differentiate) {
                                        direction = 1;
                                    }
                                    else {
                                        direction = -1;
                                    }

                                    otherMove(newLeft, newRight, direction);
                                    telemetry.addData("left", newLeft);
                                    telemetry.addData("right", newRight);
                                    if (differentiate) {
                                        telemetry.addData("skystone is on the", "right");
                                    }
                                    else {
                                        telemetry.addData("skystone is on the", "left");
                                    }

                                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                            recognition.getLeft(), recognition.getTop());
                                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                            recognition.getRight(), recognition.getBottom());
                                    telemetry.update();
                                }
                                setMotorPower(0);

                                }
                            }
                            /*else if (left > right) { // it is on the right
                                while (left > right && opModeIsActive() && Math.abs(left - right) > tolerance) {
                                    newLeft = recognition.getLeft();
                                    newRight = recognition.getRight();
                                    otherMove(newLeft, newRight, 1);
                                    telemetry.addData("left", newLeft);
                                    telemetry.addData("right", newRight);
                                    telemetry.addData("skystone is on the", "right");
                                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                            recognition.getLeft(), recognition.getTop());
                                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                            recognition.getRight(), recognition.getBottom());
                                    telemetry.update();
                                }
                                setMotorPower(0);

                            } else if (right > left) { // it is on the left
                                while (right > left && opModeIsActive() && Math.abs(left - right) > tolerance) {
                                    newLeft = recognition.getLeft();
                                    newRight = recognition.getRight();
                                    otherMove(newLeft, newRight, -1);
                                    telemetry.addData("left", newLeft);
                                    telemetry.addData("right", newRight);
                                    telemetry.addData("skystone is on the", "left");
                                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                            recognition.getLeft(), recognition.getTop());
                                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                            recognition.getRight(), recognition.getBottom());
                                    telemetry.update();

                                }
                                setMotorPower(0);
                            }


                        } else { //not a skystone, move left then go back to the start
                            testAutoMove(0, 6);
                             telemetry.addData("not a skystone", null);
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.update();
                        }
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

    }
*/


    public void moveToCenter(double left, double right) {
        double P = 0.02;
        double error = left - right;
        double speed;
        double minSpeed = 0.01;
        double maxSpeed = 0.03;
        speed = clip(P * error, minSpeed, maxSpeed);

        setDriveMotors(speed);

    }
}