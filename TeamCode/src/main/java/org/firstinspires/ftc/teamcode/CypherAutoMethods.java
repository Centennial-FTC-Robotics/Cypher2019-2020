package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

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
        setCacheMode(LynxModule.BulkCachingMode.AUTO);
        resetEncoders();

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
        loadingAuto(side, amount, false);
    }

    protected void loadingAuto(String side, int amount, boolean useSlides) {
        //releaseIntake(); //DONT U DARE UNCOMMENT THIS LINE IT WILL BREAK THE SLIDES NO TOUCH THIS
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
            //bruh i love you syke
            //dont care
            testAutoMove(0, -40);
            currentPos.add(0, convertInchToTile(40 * factor));
            waitControlIntake(.7);
            testAutoMove(12, 0);
            currentPos.add(convertInchToTile(0), 12 * factor);


            testAutoMove(0, -12);
            // moveToPos(currentPos.getX(), 4, dir); //move to other side
            if (!useSlides) {
                testAutoMove(-22.75 * 3, 0); //move to other side
                currentPos.add(0, 22.75 * 3);
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
            } else {
                moveToPos(currentPos.getX(), redFoundation.getY(), dir);
                turnRelative(180);
                dir = 180;
                moveIntakedStone();
                ElapsedTime time = new ElapsedTime();
                while (time.seconds() < 4) {

                }


            }
            if (factor == 1)
                moveToPos(new Tile(6, 5 - convertInchToTile(1d / 3), 2, 1), dir);
            else
                moveToPos(new Tile(1, 5 + convertInchToTile(1d / 3), 2, 1), dir);

            if (i == 0)
                moveToPos(currentPos.getX(), oldPos.getY(), dir);
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
        while (time.milliseconds() < 120 && opModeIsActive()) {
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
                            if (containsSkystone(updatedRecognitions)) {
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
                                        if (Math.abs(recognition.getRight() - recognition.getTop() + 125) > tolerance) {
                                            telemetry.addData("moving", "to skystone.........");
                                            if (recognition.getRight() > recognition.getTop() + 125) {
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
                            } else {
                                testAutoMove(-6, 0);
                            }

                        }
                    }
                }

            } while (!skystoneFound && timer.seconds() < 10 && opModeIsActive());
        }
    }

    private boolean containsSkystone(List<Recognition> recognitions) {
        for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT))
                return true;
        }
        return false;
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
                    stopEverything();
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

    protected void getFoundation(int factor, Side side) { //changed from protected to private, so warnings can stop yelling
        //turnRelative(180);
        testAutoMove(-30, 0);
        testAutoMove(0, -10 * factor);
        controlFoundation(FoundationState.DRAG);
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 1 && opModeIsActive()) {
            if (shouldStop())
                stopEverything();
        }
        testAutoMove(40, 0);
        turnRelative(110 * factor);
        timer.reset();
        controlFoundation(FoundationState.RELEASE);
        while (timer.seconds() < 1 && opModeIsActive()) {
            if (shouldStop())
                stopEverything();
        }
        if (side == Side.BRIDGE)
            testAutoMove(30, 0);
        else {
            testAutoMove(30, 20);
        }
        //turnRelative(90);
        //bruh its charged oh my

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

                /* TODO:
                  add part to drop off stone and move foundation
                  and park on the specified side
                  and like work in general
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

    void selfCorrectStrafe(double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        resetEncoders();
        double P = 1 / 1333d;
        double I = 0;
        double tolerance = 5;
        double angleTolerance = 10;
        double minSpeed = 0.01;
        double maxSpeed = 0.5;
        double negSpeed, posSpeed;
        double currentNegPos, currentPosPos;
        double currentAngle;
        double negError, posError;
        double negSum = 0, posSum = 0;
        double angleError;
        double startAngle = getRotationDimension();
        //double startAngle = getRotationDimension('Z');

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        do {
            if (shouldStop())
                stopEverything();
            //currentAngle = getRotationDimension('Z');
            currentAngle = getRotationDimension();
            angleError = currentAngle - startAngle;

            if (Math.abs(angleError) > angleTolerance) {
                turnAbsolute(startAngle);
            }
            if (shouldStop())
                stopEverything();

            currentNegPos = getNegPos();
            currentPosPos = getPosPos();

            negError = currentNegPos - negTarget;
            posError = currentPosPos - posTarget;

            negSum += negError;
            posSum += posError;
            negSpeed = clip(P * negError + I * negSum, minSpeed, maxSpeed);
            posSpeed = clip(P * posError + I * posSum, minSpeed, maxSpeed);

            setStrafeMotors(negSpeed, posSpeed);

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

    protected void betterSelfCorrectStrafe(double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        int negTarget = forwardMovement - leftMovement;
        int posTarget = forwardMovement + leftMovement;

        double P = 1d / 1333;
        double turnP = 1d / 2000;
        double tolerance = 1d / 3;
        double turnTolerance = 2d / 3;
        double minSpeed = 0.01;
        double minTurnSpeed = minSpeed;
        double maxSpeed = 0.5;
        double maxTurnSpeed = 0.3;
        double negError, posError;
        double negPos, posPos;
        double angleError;
        double startAngle = getRotationDimension();
        double currentAngle;
        double negSpeed, posSpeed, rotateSpeed;

        do {
            if (shouldStop())
                stopEverything();
            currentAngle = getRotationDimension();
            negPos = getNegPos();
            posPos = getPosPos();

            negError = negPos - negTarget;
            posError = posPos - posTarget;
            angleError = currentAngle - startAngle;

            negSpeed = clip(P * negError, minSpeed, maxSpeed);
            posSpeed = clip(P * posError, minSpeed, maxSpeed);
            rotateSpeed = clip(angleError * turnP, minTurnSpeed, maxTurnSpeed);
            turnStrafe(negSpeed, posSpeed, rotateSpeed);
        } while (opModeIsActive() && (Math.abs(negError) > tolerance || Math.abs(posError) > tolerance || Math.abs(angleError) > turnTolerance));
        setDriveMotors(0);
    }
   /*@Override
    protected void testAutoMove(double forward, double left) {
        if (left < forward) {
            testPIDThingy(0, left);
            testPIDThingy(forward, 0);
        } else {
            testPIDThingy(forward, 0);
            testPIDThingy(0, left);
        }
    }

    */


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

            }*/
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

    private void moveIntakedStone() {
        moveSlidesToPos(0);
        grabServo(0);
        moveSlidesToPos(200);
    }

    private void moveSlidesToPos(int pos) {
        for (DcMotorEx motor : vSlides) {
            motor.setTargetPosition(pos);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        if (pos > getVSlidePos()) {
            vLeft.setPower(-.2);
            vRight.setPower(-.2 * (1d / 5));
        } else {
            vLeft.setPower(.2);
            vRight.setPower(.2 * 1.2);
        }
    }

    private void releaseIntake() {
        moveSlidesToPos(100);
        while (opModeIsActive() && (vLeft.isBusy() || vRight.isBusy())) {
            if (shouldStop()) {
                stopEverything();
            }
        }
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive() && time.seconds() < 2) {
            if (shouldStop())
                stopEverything();
            HSlide.setPower(.5);
        }
        HSlide.setPower(0);
        moveSlidesToPos(getVSlidePos() - 100);

        while (opModeIsActive() && (vLeft.isBusy() || vRight.isBusy())) {
            if (shouldStop()) {
                stopEverything();
            }
        }
    }

    protected enum Team {
        RED, BLUE
    }

    protected enum Side {
        BRIDGE, WALL
    }
}
