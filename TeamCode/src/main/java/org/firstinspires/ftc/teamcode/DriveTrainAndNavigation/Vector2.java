package org.firstinspires.ftc.teamcode.DriveTrainAndNavigation;

public class Vector2 {
    public double x;
    public double y;

    public Vector2 plus(Vector2 vec) {
        return new Vector2(this.x + vec.x, this.y + vec.y);
    }

    public Vector2 times(double scalar) {
        return new Vector2(this.x * scalar, this.y * scalar);
    }

    public Vector2 minus(Vector2 vec) {
        return new Vector2(this.x - vec.x, this.y - vec.y);
    }

    public double dot(Vector2 vec) {
        return vec.x * vec.x + vec.y * vec.y;
    }

    public double dot(Vector2 vec1, Vector2 vec2) {
        return dot(vec1.minus(vec2));
    }
//
//    public Vector2 cross(Vector2 vec) {
//        return
//    }

    public Vector2 (double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2() {
        this.x = 0;
        this.y = 0;
    }

    public String toString() {
        return String.format("(%f, %f)", this.x, this.y);
    }
}
