package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class CypherAutoMethods extends CypherMethods {
    SkystoneDetector detector = new SkystoneDetector();

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        changeColor(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
        setCacheMode(LynxModule.BulkCachingMode.AUTO);
        resetEncoders();
        //controlFoundation(FoundationState.RELEASE); maybe resets servos???? builders would preferably want it
    }

    void initVision(LinearOpMode opMode) {
        detector.activate(opMode);
    }

    protected void initEverything() {
        initializeIMU();
        detector.activate(this);
        while (!imu.isGyroCalibrated() && !isStopRequested()) {
            if (isStopRequested())
                stopEverything();
        }
    }

    protected void loadingAuto(Team team, int amount) {
        loadingAuto(team, amount, false);
    }

    protected void loadingAuto(Team team, int amount, boolean useSlides) {
        //releaseIntake(); //DONT U DARE UNCOMMENT THIS LINE IT WILL BREAK THE SLIDES NO TOUCH THIS
        int factor = 1;
        switch (team) {
            case RED:
                factor = 1;
                break;
            case BLUE:
                factor = -1;
                break;
        }
        testAutoMove(6, -6); //move forward and strafe a bit
        for (int i = 0; i < amount; i++) {
            resetEncoders();
            //skystoneFindPls(factor); //center robot w/ skystone
            double distTravelled = convertEncoderToInch(getPos()); //find out how far we travelled to get to the skystone

            testAutoMove(-12, 0); //move backwards
            testAutoMove(0, -30); //strafe to where the skystone is

            waitControlIntake(.5); //intake!!!!!
            testAutoMove(16, 0); //move forward to intake!!!
            controlIntakeMotors(0.2); //slow down intake but keep it spinning

            testAutoMove(0, 12);//move to go towards bridge side
            // moveToPos(currentPos.getX(), 4, dir); //move to other side
            testAutoMove((TILE_LENGTH * 3) + distTravelled, 0); //move to other side
            if (!useSlides) {
                turnRelative(90);//turn
                testAutoMove(-20, 0); //bakc into foundatio
                //drag it
                waitMoveFoundation(FoundationState.DRAG);
                testAutoMove(30, 0);
                turnAbsolute(-90);
                waitMoveFoundation(FoundationState.RELEASE);
                //release stone
                waitControlIntake(-0.3);
            } else {
                turnRelative(90);
                controlIntakeMotors(0);
                //moveIntakedStone();
                waitSec(2.5);
                turnRelative(180);
                waitMoveFoundation(FoundationState.DRAG);
                testAutoMove(30, 0);
                turnAbsolute(-90);
                waitMoveFoundation(FoundationState.RELEASE);

            }
            testAutoMove(0, -20 * factor);
            if (amount == 2 && i == 0) { //if were getting 2 stones and this was our first one
                //this will matter if we ever get 2 stone auto
                //will be what the robot should do to get a 2nd stone
                //might need to improve getting the 1st stone so that the robot will know what # stone it is
                //and then figure out where the 2nd stone is from that
            }
        }
        testAutoMove(TILE_LENGTH * 2, 0); //change this if we ever do a 2 stone auto lmao


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


    private boolean isSame(double right1, double top1, double right2, double top2) {
        double rightDiff = Math.abs(right1 - right2);
        double topDiff = Math.abs(top1 - top2);
        final int TOLERANCE = 100;
        return (rightDiff <= TOLERANCE) && (topDiff <= TOLERANCE);
    }

    //what is this and why
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
        testAutoMove(25, 0);
        turnRelative(90 * factor);
        timer.reset();
        controlFoundation(FoundationState.RELEASE);
        while (timer.seconds() < 1 && opModeIsActive()) {
            if (shouldStop())
                stopEverything();
        }
        if (side == Side.BRIDGE)
            testAutoMove(40, 0);
        else {
            testAutoMove(0, 20);
            testAutoMove(40, 0);
        }
        //turnRelative(90);
        //bruh its charged oh my

    }

    void selfCorrectStrafe(double forward, double left) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        resetEncoders();
        double P = 1 / 1333d;
        double I = 0;
        double tolerance = 15;
        double angleTolerance = 7;
        double minSpeed = 0.01;
        double maxSpeed = 0.2333333333333333333333333333;
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
            //P = tuner(P);

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
        double turnP = 1d / 1000;
        double tolerance = 1d / 3;
        double turnTolerance = 5;
        double minSpeed = 0.01;
        double minTurnSpeed = minSpeed;
        double maxSpeed = 0.2;
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

            negError = negTarget - negPos;
            posError = posTarget - posPos;
            angleError = currentAngle - startAngle;

            negSpeed = clip(P * negError, minSpeed, maxSpeed);
            posSpeed = clip(P * posError, minSpeed, maxSpeed);
            rotateSpeed = clip(angleError * turnP, minTurnSpeed, maxTurnSpeed);
            turnStrafe(negSpeed, posSpeed, rotateSpeed);
        } while (opModeIsActive() && (Math.abs(negError) > tolerance || Math.abs(posError) > tolerance || Math.abs(angleError) > turnTolerance));
        setDriveMotors(0);
    }


    void getInPos(Team team) {
        int factor = 1;
        if (team == Team.RED)
            factor = -1;
        //testAutoMove(0, -(TILE_LENGTH * (2d / 3)) * factor);
        //testAutoMove(-((1d / 3) * TILE_LENGTH), 0);
        detector.orderStones();
        //testAutoMove(0, -(TILE_LENGTH * (1d / 2) * factor + 4));
        //testAutoMove(-TILE_LENGTH * 2d / 3, 0);
    }

    void moveToStone(int pos, Team team) {
        int factor = 1;
        int target = pos - 1;
        //we need to test out some values for this
        /* whats needed
        where will we tell the robot to go to see the first 2 or first 3 stones
        how far does the robot need to travel backwards to get to a stone
        how far does the robot need to strafe to knock others out of its way
        how far does the robot need to move to actually get the stone in the intake
         */
        if (team == Team.RED)
            factor = -1;
        if (pos == 1) {
            testAutoMove(TILE_LENGTH * 1d / 5, 0);
            testAutoMove(0, TILE_LENGTH * 1d / 3);
        } else {

        }


    }

    protected enum Team {
        RED, BLUE
    }

    protected enum Side {
        BRIDGE, WALL
    }

    /* why we still this IDK
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

   /* protected void buildingAuto(String side) {
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

    */
/*
    private void moveToPos(double x2, double y2, int dir) {
        moveToPos(new Tile(x2, y2), dir);
    }

    private void moveToPos(Tile end, int dir) {
        double[] move = getDist(currentPos, end, dir);
        testAutoMove(move[0], move[1]);
        currentPos.setLocation(end);
    }

 */
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
/*
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
*/
    //we don't even use this and we use own park methods so getting rid of it prob
    /*
    protected void park(Team team, Side side) {
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
*/

}
