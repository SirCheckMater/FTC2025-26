package org.firstinspires.ftc.teamcode.Utilities;

public class Angle {

    private double angle;
    private double offset;

    public Angle(double angle) {
        this.angle = angle;
        offset = 0;
    }

    public Angle(double angle, Direction direction) {
        switch (direction) {
            case RIGHT:
                offset = 0;
                break;
            case UP:
                offset = 90;
                break;
            case LEFT:
                offset = 180;
                break;
            case DOWN:
                offset = 270;
                break;
        }
        this.angle = angle + offset;
    }

    public double getDeg() {
        return angle - offset;
    }

    public double getRad() {
        return degsToRads(angle - offset);
    }

    public double sin() {
        return degSin(angle);
    }

    public double cos() {
        return degCos(angle);
    }

    public double tan() {
        return degTan(angle);
    }

    public enum Direction {
        RIGHT, UP, LEFT, DOWN
    }

    public static double degsToRads(Double theta) {
        return theta * (Math.PI / 180);
    }

    public static double radsToDegs(Double theta) {
        return theta * (180 / Math.PI);
    }


    public static double degSin(double theta) {
        return Math.sin(degsToRads(theta));
    }

    public static double degCos(double theta) {
        return Math.cos(degsToRads(theta));
    }

    public static double degTan(double theta) {
        return Math.tan(degsToRads(theta));
    }

    public static double degASin(double sin) {
        return radsToDegs(Math.asin(sin));
    }

    public static double degATan(double opposite, double adjacent) {
        return radsToDegs(Math.atan2(opposite, adjacent));
    }
}