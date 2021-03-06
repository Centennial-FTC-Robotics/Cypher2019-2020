package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class CypherAutoMethods extends CypherMethods {
    protected SkystoneDetector detector = new SkystoneDetector();
    int[] skystonePos = new int[2];

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        changeColor(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
        setCacheMode(LynxModule.BulkCachingMode.AUTO);
        resetEncoders();
        //controlFoundation(FoundationState.RELEASE); maybe resets servos???? builders would preferably want it
    }

    protected void initVision(LinearOpMode opMode) {
        detector.activate(opMode);
        telemetry.addLine("init done");
        telemetry.update();
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
        double distTravelled;
        //releaseIntake(); //DONT U DARE UNCOMMENT THIS LINE IT WILL BREAK THE SLIDES NO TOUCH THIS
        int factor = -1;
        if (team == Team.RED)
            factor = 1;

        //let the intake thingy out
        controlIntakeMotors(0.4);
        double angle = getRotationDimension();
        getInPos(team); //go get ready and scan for skystones
        double angleOther = getRotationDimension();
        skystonePos = detector.getSkystonePositions(); //find out positions for them

        //does not clear the telemetry for the stone positions!1!!11!!
        telemetry.addData("first skystone", skystonePos[0]).setRetained(true);
        telemetry.addData("second skystone", skystonePos[1]).setRetained(true);
        telemetry.update();
        distTravelled = moveToStone(skystonePos[0], team); //go get it
        turnAbsolute((angle + angleOther) / 2); //rotate back to original angle (hitting the stones messes it up a bit)
        controlIntakeMotors(0.3);
        autoMove(-(TILE_LENGTH * 3.7 + distTravelled) * factor, 0); //move to other side
        turnAbsolute(90); //turn
        //aquire foundation
        autoMove(-TILE_LENGTH * (2d / 3) + 4, 0);
        controlFoundation(FoundationState.DRAG);
        waitMilli(500);
        grabStone(ArmState.DROP);
        autoMove(TILE_LENGTH * 2, 0);
        controlIntakeMotors(-0.3);
        if(team == Team.BLUE) {
            turnAbsolute(180);
        } else
            turnAbsolute(0);
        controlIntakeMotors(0);

        autoMove(-25, 0);
        //ok who needs foundation anymore
        controlFoundation(FoundationState.RELEASE);
        waitMilli(400);

        //2nd stone time
        if (amount == 2) {
            //blah blah blah idfk code works
            //maybe
            autoMove(findDistToSkystone(skystonePos[1]) + (TILE_LENGTH * 3.8), 0);
            grabSkystone(team, 2);
            controlIntakeMotors(0.3);

            autoMove(-(findDistToSkystone(skystonePos[1]) + (TILE_LENGTH * 3.5)), 0);
            grabStone(ArmState.DROP);

            turnRelative(90); //if it breaks prob thia line right here

            controlIntakeMotors(-0.3);
            waitMilli(300);
            controlIntakeMotors(0);

            autoMove(0, -TILE_LENGTH * 1.5);
        } else {
            autoMove(40, 0);
        }

    }

    protected void waitControlIntake(double power) {
        ElapsedTime time = new ElapsedTime();
        controlIntakeMotors(power);
        while (time.milliseconds() < 40 && opModeIsActive()) {
            if (shouldStop()) {
                stopEverything();
                break;
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

    protected void getFoundation(int factor, Side side) { //changed from protected to private, so warnings can stop yelling
        //turnRelative(180);
        autoMove(-30, 0);
        autoMove(0, -10 * factor);
        controlFoundation(FoundationState.DRAG);
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 1 && opModeIsActive()) {
            if (shouldStop())
                stopEverything();
        }
        autoMove(30, 0);
        turnRelative(90 * factor);
        timer.reset();
        controlFoundation(FoundationState.RELEASE);
        autoMove(-15,0);
        while (timer.seconds() < 1 && opModeIsActive()) {
            if (shouldStop())
                stopEverything();
        }
        if (side == Side.BRIDGE)
            autoMove(40, 0);
        else {
            autoMove(0, 25);
            autoMove(40, 0);
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
        double turnP = 1d / 1222;
        double tolerance = 1d / 3;
        double turnTolerance = 10;
        double minSpeed = 0.01;
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
            if (Math.abs(angleError) > tolerance) {
                rotateSpeed = clip(angleError * turnP, minSpeed, maxTurnSpeed);
            } else {
                rotateSpeed = 0;
            }
            turnStrafe(negSpeed, posSpeed, rotateSpeed);
        } while (opModeIsActive() && (Math.abs(negError) > tolerance || Math.abs(posError) > tolerance || Math.abs(angleError) > turnTolerance));
        setDriveMotors(0);
    }

    protected void getInPos(Team team) {
        int factor = -1;
        if (team == Team.RED)
            factor = 1;
        autoMove(0, -(TILE_LENGTH * (1d / 2) + 4));
        ElapsedTime time = new ElapsedTime();
        while (time.milliseconds() < 900) {
            if (team == Team.BLUE)
                detector.determineOrder();
            else
                detector.determineOrder23();
        }
        autoMove(0, -(TILE_LENGTH * (2d / 3) - 4));
    }


    protected double moveToStone(int pos, Team team) {
        int factor = 1;
        int idkWhatToCallThis;
        double dist = 0;
        //we need to test out some values for this
        /* whats needed
        where will we tell the robot to go to see the first 2 or first 3 stones
        how far does the robot need to travel backwards to get to a stone
        how far does the robot need to strafe to knock others out of its way
        how far does the robot need to move to actually get the stone in the intake
         */
        if (team == Team.RED)
            factor = -1;
        if (pos == 3) {
            dist = -8 * factor;
        } else if (pos < 4) {
            idkWhatToCallThis = 4 - pos;
            dist = factor * (idkWhatToCallThis * 6);
        } else {
            idkWhatToCallThis = pos - 4;
            dist = -idkWhatToCallThis * 5 * factor;
        }


        if (team == Team.RED)
            autoMove(dist, 0);
        else {
            if (pos == 3) {
                autoMove(-20, 0);
                dist = -20;
            } else
                autoMove(-dist, 0);
            //Bill was here lol

        }

        grabSkystone(team,1);
        dist += -10 * factor;

        //now u can run the proper auto!!!11!!!
        return dist;
    }

    protected void grabSkystone(Team team, int num) {

        if(team == Team.BLUE && num == 1) {
            autoMove(0, -(TILE_LENGTH * (2d / 3d) + 4));
            waitControlIntake(0.7777777777777777777);
            autoMove(8, 0);
            autoMove(0, (TILE_LENGTH * (2d / 3d) + 8));
        } else if((team == Team.BLUE && num == 2))  {
            autoMove(0, ((TILE_LENGTH * (2d / 3d) + 4) * 2) - 6);
            waitControlIntake(0.7777777777777777777);
            autoMove(8, 0);
            autoMove(0, -((TILE_LENGTH * (2d / 3d) + 10) * 2) - 6);
        } else if(team == Team.RED) {
            autoMove(0, (TILE_LENGTH * (2d / 3d) + 4));
            waitControlIntake(0.7777777777777777777);
            autoMove(8, 0);
            autoMove(0, -(TILE_LENGTH * (2d / 3d) + 10));
        }



        grabStone(ArmState.PICK);

    }

    //will find the dist compared to the tile in either (5,2) or (2,2)
    //only call this when robot is facing loading zone
    protected double findDistToSkystone(int pos) {
        double dist;
        int idkWhatToCallThis;
        if (pos == 3) {
            dist = -2.5;
        } else if (pos < 4) {
            idkWhatToCallThis = 4 - pos;
            dist = -(idkWhatToCallThis * 6 + 3);
        } else {
            idkWhatToCallThis = pos - 4;
            dist = idkWhatToCallThis * 3.5;
        }
        return dist;
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
            autoMove(2, 0);
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
        autoMove(move[0], move[1]);
        currentPos.setLocation(end);
    }

 */
/*@Override
    protected void autoMove(double forward, double left) {
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
                        autoMove(30, 0);
                    case WALL:
                        autoMove(0, -10);
                        autoMove(30, 0);
                }
            case BLUE:
                switch (side) {
                    case BRIDGE:
                        autoMove(30, 0);
                    case WALL:
                        autoMove(0, 10);
                        autoMove(30, 0);
                }


        }

    }
*/

}
