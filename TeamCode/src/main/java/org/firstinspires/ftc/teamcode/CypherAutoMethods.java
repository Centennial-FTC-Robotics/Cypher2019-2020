package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;
import java.util.Queue;

public abstract class CypherAutoMethods extends CypherMethods {

    protected final Tile currentPos = new Tile(0, 0); //always start here; changed from protected to private to lose warning
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
        changeColor(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
    }


    void emergencyMove(String side, String color) {
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
        //the robot is always like that wdym
        //ok boomer
        telemetry.update();
        timer.reset();
        do {
            if (shouldStop()) {
                stopEverything();
            }
            setStrafeMotors(-0.4 * factor, 0.4 * factor);
        } while (timer.seconds() < 2 && opModeIsActive());
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
            waitControlIntake(-.5); //release skystone
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
        stopEverything();

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
        testAutoMove(0, -6);
        currentPos.add(convertInchToTile(6) * factor, 0);
        for (int i = 0; i < amount; i++) {
            resetEncoders();
            skystoneFindPls(factor);
            currentPos.add(0, -convertInchToTile(convertEncoderToInch(getPos()))); //find how far we travelled to find skystone
            Tile oldPos = new Tile(currentPos);
            resetEncoders();
            testAutoMove(-6, 0);
            testAutoMove(0, 40 * factor);
            currentPos.add(convertInchToTile(-6) * factor, convertInchToTile(40 * factor));
            waitControlIntake(.7);
            testAutoMove(3, 0);
            currentPos.add(convertInchToTile(3), 0);


            moveToPos(currentPos.getX() - factor, currentPos.getY(), dir); //move a bit to prevent hitting the neutral bridge
            moveToPos(currentPos.getX(), blueBridge.getY() + 1.5, dir); //move to other side

            turnRelative(-90 * factor); //turn to spit out block w/o it getting in way
            dir = -90 * factor; //change dir
            waitControlIntake(-.5); //spit it out

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


    }

    private void moveToPos(double x2, double y2, int dir) {
        moveToPos(new Tile(x2, y2), dir);
    }

    private void moveToPos(Tile end, int dir) {
        double[] move = getDist(currentPos, end, dir);
        testAutoMove(move[0], move[1]);
        currentPos.setLocation(end);
    }

    private void waitControlIntake(double power) {
        ElapsedTime time = new ElapsedTime();
        controlIntakeMotors(power);
        while (time.milliseconds() < 500 && opModeIsActive()) {
            if (shouldStop()) {
                stopEverything();
            }
        }
    }

    private void waitMoveFoundation(FoundationState state) {
        ElapsedTime time = new ElapsedTime();
        if (state.equals(FoundationState.DRAG)) {
            moveFoundation(0.1);
        } else {
            moveFoundation(1);
        }
        while (time.milliseconds() < 200 && opModeIsActive()) {
            if (shouldStop()) {
                stopEverything();
            }
        }
    }

    private double[] getDist(Tile start, Tile end, int dir) {
        /*
        The angle manipulation is assuming that the degrees become negative right after it crosses 180 degrees.
        For instance, this assumption implies that one degree after 180 degrees is -1 degree because 90 more then becomes -90.
        It also assumes that the degree system stays negative from -90 and 90 degrees onward from that:
         */

        /*
        Actual Attempt at angle stuff:

        int actualDir;
        double forward;
        double left;
        double endHorzShift;
        double endVertShift;
        double begHorzShift;
        double begVertShift;
        if (dir >= 0) {
            actualDir = -dir + 90;
        }
        else {
            actualDir = dir - 90;
        }

        endHorzShift = end.getX()*Math.cos(actualDir) + end.getY()*Math.sin(actualDir);
        endVertShift = -end.getX()*Math.sin(actualDir) + end.getY()*Math.cos(actualDir);
        begHorzShift = start.getX()*Math.cos(actualDir) + start.getY()*Math.sin(actualDir);
        begVertzShift = -start.getX()*Math.sin(actualDir) + start.getY()*Math.cos(actualDir);

        forward = endHorzShift - begHorzShift;
        left = endVertShift - begVertShift;

        return new double[]{tilesToInch(forward), tilesToInch(left)};

        */

        /*
        If it does not work, it's probably a result of one of the two things:
        1.) The way the angle system works, such as when it turns negative.
        2.) Switching forward and left from each other, in which case, just swap the forward and let declaration statements.
         */

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

    protected void skystoneFindPls(int factor) { //changed from protected to private, so warnings can stop yelling
        ElapsedTime timer = new ElapsedTime();
        final double tolerance = 50;
        boolean skystoneFound = false;
        double oldRight = 0, oldTop = 0;
        if (opModeIsActive()) {
            int max, counter = 0;
            do {
                if (shouldStop()) {
                    stopEverything();
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
                                stopEverything();
                            }

                            //testing and a small bit of fine tuning
                            if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT       )) {
                                if (!skystoneFound) {
                                    oldRight = recognition.getRight();
                                    oldTop = recognition.getTop();
                                    skystoneFound = true;
                                }
                                if (isSame(oldRight, oldTop, recognition.getRight(), recognition.getTop())) {
                                    telemetry.addData("SKYSTONE", true);
                                    telemetry.addData("left", recognition.getLeft());
                                    telemetry.addData("right", recognition.getRight());
                                    if (Math.abs(recognition.getRight() - recognition.getTop() + 200) > tolerance) {
                                        telemetry.addData("moving", "to skystone.........");
                                        if (recognition.getRight() > recognition.getTop() + 200) {
                                            setDriveMotors(0.1);
                                            telemetry.addData("moving", "forward");
                                        } else {
                                            setDriveMotors(-0.1);
                                            telemetry.addData("moving", "backwards");
                                        }
                                        //auTO NEEDS TO WORK BY SATERDAY!
                                        //ok boomer
                                        //alright, happy Saterday Holidays!
                                    } else {
                                        telemetry.addData("moving", "to the side.........");
                                        skystoneFound = true;
                                        break;
                                    }
                                    oldRight = recognition.getRight();
                                    oldTop = recognition.getTop();
                                }
                            } else if (counter == max) {
                                telemetry.addData("not skystone", true);
                                break;
                            }
                            telemetry.update();
                        }
                    }
                }
            } while (!skystoneFound && timer.seconds() < 10 && opModeIsActive());
        }
    }

    protected void skystonePrintPls(int factor) {
        ElapsedTime timer = new ElapsedTime();
        final double tolerance = 50;
        boolean isSkystone = false;
        boolean skystoneFound = false;
        double oldRight = 0, oldTop = 0;
        if (opModeIsActive()) {
            int max, counter = 0;
            do {
                if (shouldStop()) {

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
                                stopEverything();
                            }

                            //done needs testing and a small bit of fine tuning
                            if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                                if (!skystoneFound) {
                                    oldRight = recognition.getRight();
                                    oldTop = recognition.getTop();
                                    skystoneFound = true;
                                }
                                if (isSame(oldRight, oldTop, recognition.getRight(), recognition.getTop())) {
                                    if (Math.abs(recognition.getRight() - recognition.getTop() + 200) > tolerance) {
                                        if (recognition.getRight() > recognition.getTop() + 200)
                                            telemetry.addData("Robot would move", "forward");
                                        else
                                            telemetry.addData("Robot would move", "backwards");
                                        //auTO NEEDS TO WORK BY SATERDAY!
                                        //ok boomer
                                        //happy saterday everyone
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
                                break;
                            }
                            telemetry.update();
                        }
                    }
                }
            } while (!isSkystone && opModeIsActive());

        }
    }

    private boolean isSame(double right1, double top1, double right2, double top2) {
        double rightDiff = Math.abs(right1 - right2);
        double topDiff = Math.abs(top1 - top2);
        final int TOLERANCE = 100;
        return (rightDiff <= TOLERANCE) && (topDiff <= TOLERANCE);
    }


    protected void emergRedLoading() {
        turnRelative(-90);
        testAutoMove(34, 0);
    }

    protected void emergRedBuilding() {
        turnRelative(90);
        testAutoMove(34, 0);

    }

    protected void getFoundation(int factor) {
        getFoundation(factor, Side.BRIDGE);
    }

    protected void getFoundation(int factor, Side side) { //changed from protected to private, so warnings can stop yelling
        //turnRelative(180);
        testAutoMove(-30, 0);
        testAutoMove(0, -10);
        controlFoundation(FoundationState.DRAG);
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 1 && opModeIsActive()) {
            if(shouldStop())
                stopEverything();
        }
        testAutoMove(44, 0);
        turnRelative(75 * factor);
        timer.reset();
        controlFoundation(FoundationState.RELEASE);
        while (timer.seconds() < 1 && opModeIsActive()) {
            if(shouldStop())
                stopEverything();
        }
        if (side == Side.BRIDGE)
            testAutoMove(26 * factor, 0);
        else {
            testAutoMove(-26 * factor, 20 * factor);
        }
        //turnRelative(90);

    }

    protected void park(Team team, Side side) {
        //highly doubtful team is needed but if so put it in; also prob just need testautomove only
        switch (team) {
            case RED:
                switch (side) {
                    case BRIDGE:
                        testAutoMove(30, 0);
                    case WALL:
                        testAutoMove(0, -10);
                        testAutoMove(30, 0);
                }
            case BLUE:
                switch (side) {
                    case BRIDGE:
                        testAutoMove(30, 0);
                    case WALL:
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
                waitControlIntake(.7);
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
                  TODO: Are we still using actual auton tho? if not then put in unused methods
                 */
                moveToPos(currentPos.getX(), blueBridge.getY() + 1.5, dir); //move to other side

                turnRelative(-90 * factor); //turn to spit out block w/o it getting in way
                dir = -90 * factor; //change dir
                waitControlIntake(-.5); //spit it out

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

    }
    @Override
    protected void testAutoMove(double forward, double left) {
        if (left < forward) {
            testPIDThingy(0, left);
            testPIDThingy(forward, 0);
        } else {
            testPIDThingy(forward, 0);
            testPIDThingy(0, left);
        }
    }


    protected void testPIDThingy(double forward, double left)  {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);
        double kP = 1d/1333;
        double kI = 1d/2500;
        double kD = 0;
        double tolerance = 1d / 3;
        double deltaTime, oldTime = 0;
        double minSpeed = 0.03;
        double maxSpeed = 0.5;
        ElapsedTime runtime = new ElapsedTime();
        /*
        Value in index 0 is for the neg motors
        Value in index 1 is for the pos motors
        Using arrays for this and not different variables to stop us from having 50 different
        variables and arrays are also just cool
         */
        double[] proportional = new double[2];
        double[] integral = new double[2];
        double[] derivative = new double[2];
        double[] speed = new double[2];
        double[] pos = new double[2];
        double[] error = new double[2];
        double[] oldError = {0, 0};
        double[] target = {forwardMovement - leftMovement, forwardMovement + leftMovement};
        setCacheMode(LynxModule.BulkCachingMode.MANUAL);
        resetEncoders();
        do {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
            if(shouldStop())
                stopEverything();
            pos[0] = getNegPos();
            pos[1] = getPosPos();
            deltaTime = Math.abs(runtime.seconds() - oldTime);
            for (int i = 0; i < 2; i++) {
                error[i] = target[i] - pos[i];
                proportional[i] = kP * error[i];
                integral[i] += error[i] * deltaTime;
                derivative[i] = (oldError[i] - error[i])/ deltaTime;
                speed[i] = clip(proportional[i] + integral[i] * kI + derivative[i] * kD , minSpeed, maxSpeed);
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

    protected enum Team {
        RED, BLUE
    }

    protected enum Side {
        BRIDGE, WALL
    }
}
