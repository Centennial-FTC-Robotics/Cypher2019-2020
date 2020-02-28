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
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final float SIZE_OF_STONE = 0; //set later
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String VUFORIA_KEY = " AU4rZ23/////AAABmQabsAT5w0XtilSncDA5KR0mTpDy+NwTupFf3UHJK5uNazyphbkBUROQQ2ZmBNd5GDwgLEOA5XgeSxjo+pUUbNa85M03eRdF7I/O0083+YEIEORW45bjU4jNszzo5ASNn2Irz3QROUIg3T+1D8+H0n3AAt4ZL3f4P/zs+NsXPhaAhsE0lVn8EMEuXZm0jMoNhwp/cHISVhb0c4ZMywtCwMYR61l2oJLEvxIQmMC6AzKi2W8Ce+W8a2daBITha+t4FCLQgKCGTZG65/I24bdwW6aNt+Yd3HltnWnl13IKdZ5xJ0DDdM5i6x/8oMoqQfPxbOVnQez4dio31wAi7B23d42Ef2yJzTTRh1YFCRoy2aJY";
    Stone firstSkystone, secondSkystone;
    List<Stone> skystones = new ArrayList<>();
    LinearOpMode opMode;
    public CypherAutoMethods.Team team;
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    boolean isUncertain = false;
    int[] possiblePos = new int[2];

    void activate(LinearOpMode opMode) {
        this.opMode = opMode;
        initVuforia();
        initTfod();

        if (tfod != null)
            tfod.activate();
    }

    void debugStuff() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null) {
                for (Recognition recognition : recognitions) {
                    opMode.telemetry.addData("size", findSize(recognition));
                }
                opMode.telemetry.update();
            }
        }
    }

    void setTeam(CypherAutoMethods.Team team) {
        this.team = team;
    }

    void debug() {
        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        if (recognitions != null) {
            telemetry.addData("# detected", recognitions.size());
            float diff;
            for (Recognition recognition : recognitions) {
                diff = findDiff(recognition);
                telemetry.addData(recognition.getLabel(), diff);
            }
        }
        telemetry.update();
    }

    //figures out position of skystones assuming it can see at least the first 2 stones
    void orderStones() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null) {
                List<Stone> pos = new ArrayList<>();
                if (recognitions.size() == 2) {
                    determineOrder(recognitions);
                } else {
                    for (Recognition recognition : recognitions) {
                        pos.add(new Stone(recognition));
                    }
                    findDupes(pos);
                    Collections.sort(pos);
                    int i = 1;
                    for (Stone stone : pos) {
                        if (stone.isSkystone) {
                            stone.setPos(i);
                            firstSkystone = stone;
                            break;
                        }
                        i++;
                    }
                    checkCertainty(pos);
                    secondSkystone = findOther(firstSkystone);
                }
            }
        }
    }


    private void checkCertainty(List<Stone> stones) {
        float currentSize, oldSize;
        oldSize = findSize(stones.get(0).recognition);
        for (Stone stone : stones) {
            currentSize = findSize(stone.recognition);
            if (oldSize == currentSize && stone.isSkystone) {
                isUncertain = true;
                possiblePos[0] = stone.pos + 1;
                possiblePos[1] = findOther(stone).pos + 1;
                break;
            }
            oldSize = currentSize;
        }
    }

    private void determineOrder(List<Recognition> recognitions) {
        if (!containsSkystone(recognitions)) {
            firstSkystone = new Stone(3, true);
        } else {
            float skystoneDiff = 0, regStoneDiff = 0;
            for (Recognition recognition : recognitions) {
                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                    skystoneDiff = findDiff(recognition);
                } else {
                    regStoneDiff = findDiff(recognition);
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


    private float findDiff(Recognition recognition) {
        if (team == CypherAutoMethods.Team.BLUE)
            return recognition.getTop() - recognition.getRight();
        else
            return recognition.getRight() - recognition.getTop();
    }

    private float findSize(Recognition recognition) {
        return Math.abs(recognition.getTop() - recognition.getBottom());
    }


    //call this if it cna see pos 2 and pos 3
    protected void determineOrder23() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null) {
                if (!containsSkystone(recognitions)) {
                    firstSkystone = new Stone(1, true);
                } else {
                    float skystoneDiff = 0, regDiff = 0;
                    for (Recognition recognition : recognitions) {
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT))
                            skystoneDiff = findDiff(recognition);
                        else
                            regDiff = findDiff(recognition);
                    }
                    if (skystoneDiff > regDiff)
                        firstSkystone = new Stone(3, true);
                    else
                        firstSkystone = new Stone(2, true);
                }
                secondSkystone = findOther(firstSkystone);
                skystones.addAll(new ArrayList<>(Arrays.asList(firstSkystone, secondSkystone)));
            }
        }
    }


    //call this if it can see pos 3 and 4
    protected void determineOrderMiddle() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null) {
                if (!containsSkystone(recognitions)) {
                    firstSkystone = new Stone(2, true);
                } else {
                    float skystoneDiff = 0, regDiff = 0;
                    for (Recognition recognition : recognitions) {
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT))
                            skystoneDiff = findDiff(recognition);
                        else
                            regDiff = findDiff(recognition);
                    }
                    if (skystoneDiff > regDiff)
                        firstSkystone = new Stone(1, true);
                    else
                        firstSkystone = new Stone(3, true);
                }

                secondSkystone = findOther(firstSkystone);
                skystones.addAll(new ArrayList<>(Arrays.asList(firstSkystone, secondSkystone)));
            }
        }
    }


    public int[] getSkystonePositions() {
        return new int[]{firstSkystone.pos, secondSkystone.pos};
    }

    private List<Stone> findDupes(List<Stone> stones) {
        List<Stone> dupes = new ArrayList<>();
        for (Stone stone : stones) {
            if (Math.abs(findSize(stone.recognition) - SIZE_OF_STONE) > 150) {
                dupes.add(new Stone(stone.recognition)); //maybe find a way to add to the distance of the other stone to make it better?
            }
        }

        for (Stone stone : dupes) {
            stones.add(new Stone(stone.recognition));
        }
        return stones;
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
            tfodParameters.minimumConfidence = 0.8;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        } else {
            stopEverything();
        }
    }

    private Stone findOther(Stone stone) {
        if (stone.pos > 3)
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
            isSkystone = recognition.getLabel().equals(LABEL_SECOND_ELEMENT);
        }

        Stone(boolean isSkystone) {
            this.isSkystone = isSkystone;
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
