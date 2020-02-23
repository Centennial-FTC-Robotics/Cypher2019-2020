package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class SkystoneDetector extends CypherMethods {
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String VUFORIA_KEY = " AU4rZ23/////AAABmQabsAT5w0XtilSncDA5KR0mTpDy+NwTupFf3UHJK5uNazyphbkBUROQQ2ZmBNd5GDwgLEOA5XgeSxjo+pUUbNa85M03eRdF7I/O0083+YEIEORW45bjU4jNszzo5ASNn2Irz3QROUIg3T+1D8+H0n3AAt4ZL3f4P/zs+NsXPhaAhsE0lVn8EMEuXZm0jMoNhwp/cHISVhb0c4ZMywtCwMYR61l2oJLEvxIQmMC6AzKi2W8Ce+W8a2daBITha+t4FCLQgKCGTZG65/I24bdwW6aNt+Yd3HltnWnl13IKdZ5xJ0DDdM5i6x/8oMoqQfPxbOVnQez4dio31wAi7B23d42Ef2yJzTTRh1YFCRoy2aJY";
    static final String LABEL_FIRST_ELEMENT = "Stone";
    static final String LABEL_SECOND_ELEMENT = "Skystone";
    Stone firstSkystone, secondSkystone;

    List<Stone> skystones = new ArrayList<>();

    LinearOpMode opMode;
    void activate(LinearOpMode opMode) {
        this.opMode = opMode;
        initVuforia();
        initTfod();

        if(tfod != null)
            tfod.activate();
    }

    //this should be called when the robot can only see the stones closest to the alliance bridge
    //will make others when it can see the first like 3 stones
    void debug() {
        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        if(recognitions != null) {
            telemetry.addData("# detected", recognitions.size());
            float diff;
            for(Recognition recognition : recognitions) {
                diff = recognition.getTop() - recognition.getRight();
                telemetry.addData(recognition.getLabel(), diff);
            }
        }
    }
    void determineOrder() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null) {
                if (!containsSkystone(recognitions)) {
                    firstSkystone = new Stone(3, true);
                } else {
                    float skystoneDiff = 0, regStoneDiff = 0;
                    for (Recognition recognition : recognitions) {
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            skystoneDiff = recognition.getTop() - recognition.getRight();
                        } else {
                            regStoneDiff = recognition.getTop() - recognition.getRight();
                        }
                    }
                    if (skystoneDiff > regStoneDiff) {
                        firstSkystone = new Stone(2, true);
                    } else {
                        firstSkystone = new Stone(1, true);
                    }
                }
                secondSkystone = findOther(firstSkystone);
                skystones.addAll(new ArrayList<>(Arrays.asList(firstSkystone, secondSkystone)));
            }
        }
    }

    int[] getSkystonePositions() {
        return new int[] {firstSkystone.pos, secondSkystone.pos};
    }

    //should be able to sort thru 3 stones at a time and figure out position
    private void orderStones(List<Recognition> recognitions) {
        List<Stone> pos = new ArrayList<>();
        for(Recognition recognition : recognitions) {
            pos.add(new Stone(recognition));
        }
        Collections.sort(pos);

        int i = 1;
        for(Stone stone : pos) {
            if (stone.isSkystone) {
                stone.setPos(i);
                firstSkystone = stone;
                break;
            }
            i++;
        }
        secondSkystone = findOther(firstSkystone);
    }

    private boolean containsSkystone(List<Recognition> recognitions) {
        for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT))
                return true;
        }
        return false;
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        if (!isStopRequested())
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        else
            stopEverything();
    }


    private void initTfod() {
        if (!isStopRequested()) {
            int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.6;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        } else {
            stopEverything();
        }
    }

    private Stone findOther(Stone stone) {
        if(stone.pos > 3)
            return new Stone(stone.pos - 3, true);
        return new Stone(stone.pos + 3, true);
    }



    private static class Stone implements Comparable<Stone> {
        int pos; //1 is closest to team bridge
        boolean isSkystone;
        float dist;
        Recognition recognition;

        Stone(int pos, boolean isSkystone) {
            this.pos = pos;
            this.isSkystone = isSkystone;
        }

        Stone(Recognition recognition) {
            this.recognition = recognition;
            dist = recognition.getTop() - recognition.getRight();
            isSkystone = recognition.getLabel().equals("Skystone");
        }

        void setPos(int pos) {
            this.pos = pos;
        }

        @Override
        public int compareTo(Stone stone) {
            int compareDist = (int) stone.dist;
            return (compareDist - (int) this.dist);
        }
    }

}
