package org.firstinspires.ftc.teamcode;

public class Tile {
    double x,y;


    Tile(double x, double y) {
        setLocation(x, y);
    }

    Tile(Tile tile) {
        setLocation(tile);
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

    void setLocation(Tile tile) {
        this.x = tile.getX();
        this.y = tile.getY();
    }

    void add(double x, double y) {
        this.x += x;
        this.y += y;
    }




}