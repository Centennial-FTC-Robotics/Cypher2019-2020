package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public abstract class CypherAutoMethods extends CypherMethods {

    protected final Tile currentPos = new Tile(0, 0); //always start here
    private final Tile redFoundation = new Tile(5, 5, 1, 3);
    private final Tile redBuildSite = new Tile(6, 6, 2, 1);
    private final Tile redQuarry = new Tile(5, 2, 3, 2);
    private final Tile redBridge = new Tile(5, 3, 2, 3);
    private final Tile blueFoundation = new Tile(redFoundation.flip());
    private final Tile blueBuildSite = new Tile(redBuildSite.flip());
    private final Tile blueQuarry = new Tile(redQuarry.flip());
    private final Tile blueBridge = new Tile(redBridge.flip());

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    void emergencyMove(String side, String color) throws StopException {
        ElapsedTime timer = new ElapsedTime();
        double factor;

        if (side.equals("loading")) {
            if (color.equals("red")) factor = 1;
            else factor = -1;
        } else {
            if (color.equals("red")) factor = -1;
            else factor = 1;

        }
        telemetry.addData("EMERGENCY", "ROBOT DOES NOT WORK NORMALLY");
        telemetry.update();
        timer.reset();
        do {
            if (shouldStop()) {
                throw new StopException("stap");
            }
            setStrafeMotors(-0.4 * factor, 0.4 * factor);
        } while (timer.seconds() < 2);
        setDriveMotors(0);
    }

    protected void buildingAuto(String side) {
        int factor = 1;
        switch (side) {
            case "red":
                factor = 1;
                break;
            case "blue":
                factor = -1;
                break;
        }
        Tile oldPos;
        try {
            moveToPos(currentPos.getX() - .5 * factor, currentPos.getY(), dir); //move forward a small bit
            turnRelative(180); //turn around so we can pick up foundation
            dir = 90 * factor;
            if (factor == 1)
                moveToPos(redFoundation, dir); //go to foundation
            else
                moveToPos(blueFoundation, dir);
            waitMoveFoundation(FoundationState.DRAG);

            if (factor == 1)
                moveToPos(redBuildSite, dir);
            else
                moveToPos(blueBuildSite, dir);
            waitMoveFoundation(FoundationState.RELEASE);
            moveFoundation(0);
            if (factor == 1)
                moveToPos(redQuarry, dir); //go to red quarry
            else
                moveToPos(blueQuarry, dir);
            turnRelative(-90 * factor);
            dir = 180;

            for (int i = 0; i < 2; i++) { //repeat twice for 2 skystones

                skystoneFindPls(factor); //center with skystone
                currentPos.add(0, -convertInchToTile(convertEncoderToInch(getPos()))); //find how far we travelled to find skystone
                oldPos = new Tile(currentPos);
                moveToPos(currentPos.getX() - convertInchToTile(factor), currentPos.getY(), dir); //move to be right behind/infront/whatever of skystone

                //pick up skystone and move into it
                controlIntakeMotors(1);
                testAutoMove(2, 0);
                currentPos.add(0, convertInchToTile(-2));

                moveToPos(currentPos.getX() + factor, currentPos.getY(), dir); //move a bit to prevent hitting the neutral bridge
                moveToPos(currentPos.getX(), currentPos.getY() + 2, dir); //move to other side

                turnRelative(-90 * factor); //turn to release skystone and not have it in the way
                dir = -90 * factor;
                waitControlIntake(-1); //release skystone
                controlIntakeMotors(0);
                turnRelative(90 * factor); //turn back
                dir = 180;
                if (i == 0) //if that was the first skystone move back to where we got the first one to look for second ome
                    moveToPos(oldPos, dir);
            }
            if (factor == 1)
                moveToPos(redBridge, dir); //go to red bridge
            else
                moveToPos(blueBridge, dir); //or blue bridge
        } catch (StopException e) {
            stopEverything();
        }
    }

    protected void loadingAuto(String side, int amount) {
        int factor = 1;
        switch (side) {
            case "red":
                factor = 1;
                break;
            case "blue":
                factor = -1;
                break;
        }
        try {
            testAutoMove(0, -6);
            currentPos.add(convertInchToTile(6) * factor, 0);
            for (int i = 0; i < amount; i++) {
                resetEncoders();
                skystoneFindPls(factor);
                currentPos.add(0, -convertInchToTile(convertEncoderToInch(getPos()))); //find how far we travelled to find skystone
                Tile oldPos = new Tile(currentPos);
                resetEncoders();
                testAutoMove(-6, 0);
                testAutoMove(0, 36 * factor);
                currentPos.add(convertInchToTile(-6) * factor, convertInchToTile(-6));
                waitControlIntake(1);
                testAutoMove(3, 0);
                currentPos.add(convertInchToTile(3), 0);


                moveToPos(currentPos.getX() + factor, currentPos.getY(), dir); //move a bit to prevent hitting the neutral bridge
                moveToPos(currentPos.getX(), blueBridge.getY() + 1.5, dir); //move to other side

                turnRelative(-90 * factor); //turn to spit out block w/o it getting in way
                dir = -90 * factor; //change dir
                waitControlIntake(-1); //spit it out

                turnRelative(180);
                dir *= -1;
                if (i == 0) { //if its the first skystone move foundation
                    if (factor == 1) {
                        moveToPos(redFoundation, dir);

                    } else {
                        moveToPos(blueFoundation, dir);
                    }
                    waitMoveFoundation(FoundationState.DRAG);
                    if (factor == 1) {
                        moveToPos(redBuildSite, dir);
                    } else {
                        moveToPos(blueBuildSite, dir);
                    }
                    waitMoveFoundation(FoundationState.RELEASE);
                }
                if (factor == 1)
                    moveToPos(new Tile(6, 5 - convertInchToTile(1d / 3), 2, 1), dir);
                else
                    moveToPos(new Tile(1, 5 + convertInchToTile(1d / 3), 2, 1), dir);

                if (i == 0) {
                    moveToPos(currentPos.getX(), oldPos.getY(), dir);
                }

                turnRelative(90 * factor);
                dir = 180;
            }
            if (factor == 1) {
                moveToPos(currentPos.getX(), redBridge.getY(), dir);
            } else {
                moveToPos(currentPos.getX(), blueBridge.getY(), dir);
            }
        } catch (StopException e) {
            stopEverything();
        }

    }

    private void moveToPos(double x2, double y2, int dir) throws StopException {
        moveToPos(new Tile(x2, y2), dir);
    }

    private void moveToPos(Tile end, int dir) throws StopException {
        double[] move = getDist(currentPos, end, dir);
        testAutoMove(move[0], move[1]);
        currentPos.setLocation(end);
    }

    private void waitControlIntake(double power) throws StopException {
        ElapsedTime time = new ElapsedTime();
        controlIntakeMotors(power);
        while (time.milliseconds() < 500) {
            if (shouldStop()) {
                throw new StopException("stap");
            }
        }
    }

    private void waitMoveFoundation(FoundationState state) throws StopException {
        ElapsedTime time = new ElapsedTime();
        if (state.equals(FoundationState.DRAG)) {
            moveFoundation(0.1);
        } else {
            moveFoundation(1);
        }
        while (time.milliseconds() < 200) {
            if (shouldStop()) {
                throw new StopException("stap");
            }
        }
    }

    private double[] getDist(Tile start, Tile end, int dir) {
        double forward, left;
        switch (dir) {
            case 90:
                forward = end.getX() - start.getX();
                left = end.getY() - start.getY();
                break;
            case 180:
                forward = start.getY() - end.getY();
                left = end.getX() - start.getX();
                break;
            case -90:
                forward = start.getX() - end.getX();
                left = end.getY() - start.getY();
                return new double[]{tilesToInch(forward), tilesToInch(left)};
            default:
                forward = end.getY() - start.getY();
                left = end.getX() - start.getX();
                break;
        }
        return new double[]{tilesToInch(forward), tilesToInch(left)};
    }

    protected void skystoneFindPls(int factor) throws StopException {
        ElapsedTime timer = new ElapsedTime();
        final double tolerance = 50;
        boolean isSkystone = false;
        boolean skystoneFound = false;
        double oldRight = 0, oldTop = 0;
        if (opModeIsActive()) {
            int max, counter = 0;
            do {
                if (shouldStop()) {
                    throw new StopException("stap");
                }
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        max = updatedRecognitions.size();
                        for (Recognition recognition : updatedRecognitions) {
                            counter++;
                            if (shouldStop()) {
                                throw new StopException("stap");
                            }

                            //testing and a small bit of fine tuning
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                if (!skystoneFound) {
                                    oldRight = recognition.getRight();
                                    oldTop = recognition.getTop();
                                    skystoneFound = true;
                                }
                                if (isSame(oldRight, oldTop, recognition.getRight(), recognition.getTop())) {
                                    telemetry.addData("SKYSTONE", true);
                                    telemetry.addData("left", recognition.getLeft());
                                    telemetry.addData("right", recognition.getRight());
                                    if (Math.abs(recognition.getRight() - recognition.getTop() + 50) > tolerance) {
                                        if (recognition.getRight() > recognition.getTop() + 150) {
                                            setDriveMotors(0.1);
                                        }
                                        else {
                                            setDriveMotors(-0.1);
                                        }
                                        //auTO NEEDS TO WORK BY SATERDAY!
                                        //ok boomer
                                        //alright, happy Saterday Holidays!
                                        telemetry.addData("moving", "to skystone.........");
                                    } else {
                                        telemetry.addData("moving", "to the side.........");
                                        isSkystone = true;
                                        break;
                                    }
                                    oldRight = recognition.getRight();
                                    oldTop = recognition.getTop();
                                }
                            } else if (counter == max) {
                                telemetry.addData("not skystone", true);
                                testAutoMove(6, 0);
                                break;
                            }
                            telemetry.update();
                        }
                    }
                }
            } while (!skystoneFound);
        }
    }

    protected void skystonePrintPls(int factor) throws StopException {
        ElapsedTime timer = new ElapsedTime();
        final double tolerance = 50;
        boolean isSkystone = false;
        boolean skystoneFound = false;
        double oldRight = 0, oldTop = 0;
        if (opModeIsActive()) {
            int max, counter = 0;
            do {
                if (shouldStop()) {
                    throw new StopException("stap");
                }
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        // step through the list of recognitions and display boundary info.
                        max = updatedRecognitions.size();
                        for (Recognition recognition : updatedRecognitions) {
                            counter++;
                            if (shouldStop()) {
                                throw new StopException("stap");
                            }

                            //done needs testing and a small bit of fine tuning
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                if (!skystoneFound) {
                                    oldRight = recognition.getRight();
                                    oldTop = recognition.getTop();
                                    skystoneFound = true;
                                }
                              if (isSame(oldRight, oldTop, recognition.getRight(), recognition.getTop())) {
                                    if (Math.abs(recognition.getRight() - recognition.getTop() + 50) > tolerance) {
                                        if (recognition.getRight() > recognition.getTop() + 150)
                                            telemetry.addData("Robot would move","forward");
                                        else
                                            telemetry.addData("Robot would move","backwards");
                                        //auTO NEEDS TO WORK BY SATERDAY!
                                        //ok boomer
                                        telemetry.update();
                                    } else {
                                        telemetry.addData("robot", "is centered");
                                        telemetry.update();
                                        isSkystone = true;
                                        break;
                                    }
                                    oldRight = recognition.getRight();
                                    oldTop = recognition.getTop();
                                } else {
                                  telemetry.addData("not the same", "u stoopid");
                                  telemetry.update();
                              }
                            } else if (counter == max) {
                                telemetry.addData("not skystone", true);
                                testAutoMove(6, 0);
                                break;
                            }
                            telemetry.update();
                        }
                    }
                }
            } while (!isSkystone);

        }
    }

    private boolean isSame(double right1, double top1, double right2, double top2) {
        double rightDiff = Math.abs(right1 - right2);
        double topDiff = Math.abs(top1 - top2);
        final int TOLERANCE = 100;
        return (rightDiff <= TOLERANCE) && (topDiff <= TOLERANCE);
    }


    protected void emergRedLoading() {
        try {
            turnRelative(-90);
            testAutoMove(34, 0);
        } catch (StopException e) {
            stopEverything();
        }
    }

    protected void emergRedBuilding() {
        try {
            turnRelative(90);
            testAutoMove(34, 0);
        } catch (StopException e) {
            stopEverything();
        }
    }

    protected void getFoundation(int factor) {
        getFoundation(factor, Side.BRIDGE);
    }
    protected void getFoundation(int factor, Side side) {
        try {
            //turnRelative(180);
            testAutoMove(-30, 0);
            testAutoMove(0, -10);
            controlFoundation(FoundationState.DRAG);
            ElapsedTime timer = new ElapsedTime();
            while (timer.seconds() < 1) ;
            testAutoMove(44, 0);
            turnRelative(75 * factor);
            timer.reset();
            controlFoundation(FoundationState.RELEASE);
            while (timer.seconds() < 1) ;
            if (side == Side.BRIDGE)
                testAutoMove(26 * factor, 0);
            else {
                testAutoMove(-26 * factor, 20 * factor);
            }
            //turnRelative(90);
        } catch (StopException e) {
            stopEverything();
            //no u
        }
    }

    protected void park(Team team, Side side) throws StopException {
        if (side == Side.BRIDGE) {
            testAutoMove(30, 0);
        } else {
            if (side == Side.LEFTWALL) {
                testAutoMove(0, -10);
                testAutoMove(30, 0);
            } else {
                testAutoMove(0, 10);
                testAutoMove(30, 0);
            }
        }

    }

    protected void actualAuto(Team team, Side side, int amount) {
        controlIntakeMotors(1);
        waitMili(25);
        controlIntakeMotors(0);
        int factor = 1;
        if (team == Team.BLUE) {
            factor = -1;
        }
        try {
            //move forward a small bit so the robot can see the stones
            currentPos.add(convertInchToTile(12) * factor, 0); //add the amount we travelled to the trash thing that holds current pos
            //for the amount of stones were supposed to find (should be 2 max)
            for (int i = 0; i < amount; i++) {
                //reset the encoders - needed for finding out how much we travelled while running finding skystone
                resetEncoders();
                skystoneFindPls(factor); //go find the skystone
                currentPos.add(0, -convertInchToTile(convertEncoderToInch(getPos()))); //find how far we travelled to find skystone and add it
                Tile oldPos = new Tile(currentPos); //save it to a new variable so if were getting the 2nd skystone we know where to start looking
                resetEncoders(); //reset the encoders - needed since were gonna use the encoder thing again
                //move backwards and to the side so the stone is in front of the robot
                testAutoMove(0, -60 * factor);
                currentPos.add(convertInchToTile(-36) * factor, convertInchToTile(-6)); //add it to the position of the robot
                //grab that stone and move forward so its actually grabbed
                waitControlIntake(1);
                testAutoMove(3, 0);
                //add that to the current pos
                currentPos.add(convertInchToTile(-3), 0);

                //if we want to just use the bridge side
                if (side == Side.BRIDGE) {
                    if (team == Team.RED)
                        moveToPos(5, currentPos.getY(), dir); //move to above the bridge side on red
                    else
                        moveToPos(2, currentPos.getY(), dir); //move to above the bridge side on blue
                } else {
                    if (team == Team.RED)
                        moveToPos(6, currentPos.getY(), dir); //move to above the wall side on red
                    else
                        moveToPos(1, currentPos.getY(), dir); //move to above the wall side on red
                }

                /*TODO: add part to drop off stone and move foundation
                  TODO: and park on the specified side
                  TODO: and like work in general
                 */
                moveToPos(currentPos.getX(), blueBridge.getY() + 1.5, dir); //move to other side

                turnRelative(-90 * factor); //turn to spit out block w/o it getting in way
                dir = -90 * factor; //change dir
                waitControlIntake(-1); //spit it out

                turnRelative(180);
                dir *= -1;
                if (i == 0) { //if its the first skystone move foundation
                    if (factor == 1) {
                        moveToPos(redFoundation, dir);
                    } else {
                        moveToPos(blueFoundation, dir);
                    }
                    waitMoveFoundation(FoundationState.DRAG);
                    if (factor == 1) {
                        moveToPos(redBuildSite, dir);
                    } else {
                        moveToPos(blueBuildSite, dir);
                    }
                    waitMoveFoundation(FoundationState.RELEASE);
                }
                if (factor == 1)
                    moveToPos(new Tile(6, 5 - convertInchToTile(1d / 3), 2, 1), dir);
                else
                    moveToPos(new Tile(1, 5 + convertInchToTile(1d / 3), 2, 1), dir);

                if (i == 0) {
                    moveToPos(currentPos.getX(), oldPos.getY(), dir);
                }

                turnRelative(90 * factor);
                dir = 180;
            }
            if (factor == 1) {
                moveToPos(currentPos.getX(), redBridge.getY(), dir);
            } else {
                moveToPos(currentPos.getX(), blueBridge.getY(), dir);
            }
        } catch (StopException e) {
            stopEverything();
        }

    }


    protected enum Team {
        RED, BLUE
    }

    protected enum Side {
        BRIDGE, LEFTWALL
    }
}
