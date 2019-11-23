package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "Tensor Flow Test", group = "Test")
public class tensorFlowTest extends CypherMethods {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    private static final String VUFORIA_KEY =
            " AU4rZ23/////AAABmQabsAT5w0XtilSncDA5KR0mTpDy+NwTupFf3UHJK5uNazyphbkBUROQQ2ZmBNd5GDwgLEOA5XgeSxjo+pUUbNa85M03eRdF7I/O0083+YEIEORW45bjU4jNszzo5ASNn2Irz3QROUIg3T+1D8+H0n3AAt4ZL3f4P/zs+NsXPhaAhsE0lVn8EMEuXZm0jMoNhwp/cHISVhb0c4ZMywtCwMYR61l2oJLEvxIQmMC6AzKi2W8Ce+W8a2daBITha+t4FCLQgKCGTZG65/I24bdwW6aNt+Yd3HltnWnl13IKdZ5xJ0DDdM5i6x/8oMoqQfPxbOVnQez4dio31wAi7B23d42Ef2yJzTTRh1YFCRoy2aJY";


    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    double tolerance = 200; //close enough value
    double newRight, newLeft;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initVuforia();
        initTfod();


        if (tfod != null) {
            tfod.activate();
        }


        waitForStart();
        //findSkystone();
        goToSkystone();

    }

    public void goToSkystone() {
        while(opModeIsActive()) {
            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update();
                    int i = 0;

                    for (Recognition recognition : updatedRecognitions) {
                        double left = recognition.getLeft();
                        double right = recognition.getRight();
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {  //if skystone is detected
                            if(left - right > tolerance) { //if its not "close enough"
                                moveToCenter(left, right); //run the thingy that makes it "close enough"
                            } else {
                                setMotorPower(0); //when u r close enough stop moving
                                testAutoMove(60, 0); //if you are close enough move forward and ram into it
                            }

                    } else { //if not skystone
                            testAutoMove(0,12); //move to side
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
                                testAutoMove(-12, 0); //negative makes it move forward
                            } else if (left > right) { // it is on the right
                                while (left > right && opModeIsActive() && Math.abs(left - right) > tolerance) {
                                    newLeft = recognition.getLeft();
                                    newRight = recognition.getRight();
                                    otherMove(newLeft, newRight, 1);
                                    telemetry.addData("left", newLeft);
                                    telemetry.addData("right", newRight);
                                    telemetry.addData("skystone is on the", "right");
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
                                    telemetry.addData("skystone is on the", "right");
                                    telemetry.update();
                                }
                                setMotorPower(0);
                            }


                        } else { //not a skystone, move left then go back to the start
                            testAutoMove(0, 6);
                            telemetry.addData("not a skystone", null);
                            telemetry.update();
                        }
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.update(); //somehow not  updating
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

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

    public void otherMove(double left, double right, double dir) { //dir = 1 is right, dir = -1 is left
        double P = 0.02;
        double error = Math.abs(left - right);
        double speed;
        double negSpeed, posSpeed;
        double minSpeed = 0.01;
        double maxSpeed = 0.3;

        speed = Range.clip(P * error, minSpeed, maxSpeed);
        negSpeed = -(speed * dir);
        posSpeed = speed * dir;

        for (DcMotor motor : strafeNeg) {
            motor.setPower(negSpeed);
        }
        for (DcMotor motor : strafePos) {
            motor.setPower(posSpeed);
        }
    }

    public void moveToCenter(double left, double right) {
        double P = 0.02;
        double error = left - right;
        double negSpeed, posSpeed;
        double minSpeed = 0.01;
        double maxSoeed = 0.03;

        negSpeed = Range.clip(-(P*error), minSpeed, maxSoeed);
        posSpeed = Range.clip(P*error, minSpeed, maxSoeed);

        setStrafeMotors(negSpeed, posSpeed);
    }
}