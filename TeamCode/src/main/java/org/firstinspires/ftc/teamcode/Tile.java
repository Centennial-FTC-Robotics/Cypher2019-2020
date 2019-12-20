package org.firstinspires.ftc.teamcode;

public class Tile {
    double x,y;


    Tile(double x, double y) {
        setLocation(x, y);
    }

    static double[] getDist(Tile start, Tile end) {
        double forward = end.getY() - start.getY();
        double left = end.getX() - start.getX();
        return new double[]{forward, left};
    }

    static double tilesToInch(double tiles) {
        return tiles*24;
    }

    double getX() {
        return x;
    }
    double getY() {
        return y;
    }

    void setLocation(double x, double y) {
        this.x = x;
        this.y = y;
    }




}
