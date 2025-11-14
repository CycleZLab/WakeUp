package org.firstinspires.ftc.teamcode.mathUtil;

import androidx.annotation.NonNull;

import java.util.Objects;

public class PolarVector {
    private final double radius;
    private final double theta;

    public static final double PI = Math.PI;
    public static final double TWO_PI = 2 * Math.PI;

    public PolarVector(double radius, double theta) {
        if (radius < 0) {
            this.radius = -radius;
            this.theta = normalizeAngle(theta + PI);
        } else {
            this.radius = radius;
            this.theta = normalizeAngle(theta);
        }
    }

    public static PolarVector fromCartesian(double x, double y) {
        double radius = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y, x);
        return new PolarVector(radius, theta);
    }

    public double getRadius() {
        return radius;
    }

    public double getTheta() {
        return theta;
    }

    public double getThetaDegrees() {
        return Math.toDegrees(theta);
    }

    public double toCartesianX() {
        return radius * Math.cos(theta);
    }

    public double toCartesianY() {
        return radius * Math.sin(theta);
    }

    public PolarVector add(PolarVector other) {
        double x1 = this.toCartesianX();
        double y1 = this.toCartesianY();
        double x2 = other.toCartesianX();
        double y2 = other.toCartesianY();

        return PolarVector.fromCartesian(x1 + x2, y1 + y2);
    }

    public PolarVector subtract(PolarVector other) {
        double x1 = this.toCartesianX();
        double y1 = this.toCartesianY();
        double x2 = other.toCartesianX();
        double y2 = other.toCartesianY();

        return PolarVector.fromCartesian(x1 - x2, y1 - y2);
    }

    public PolarVector multiply(double scalar) {
        return new PolarVector(radius * scalar, theta);
    }

    public double dotProduct(PolarVector other) {
        double x1 = this.toCartesianX();
        double y1 = this.toCartesianY();
        double x2 = other.toCartesianX();
        double y2 = other.toCartesianY();

        return x1 * x2 + y1 * y2;
    }

    public double crossProduct(PolarVector other) {
        double x1 = this.toCartesianX();
        double y1 = this.toCartesianY();
        double x2 = other.toCartesianX();
        double y2 = other.toCartesianY();

        return x1 * y2 - y1 * x2;
    }

    public PolarVector rotate(double angle) {
        return new PolarVector(radius, normalizeAngle(theta + angle));
    }

    public PolarVector reverse() {
        return new PolarVector(radius, normalizeAngle(theta + PI));
    }

    private double normalizeAngle(double angle) {
        angle = angle % TWO_PI;
        if (angle < 0) {
            angle += TWO_PI;
        }

        if (angle > PI) {
            angle -= TWO_PI;
        }
        return angle;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        PolarVector that = (PolarVector) obj;
        return Double.compare(that.radius, radius) == 0 &&
                Double.compare(that.theta, theta) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(radius, theta);
    }

    @Override
    public String toString() {
        return String.format("PolarVector(r=%.3f, θ=%.3f rad, %.3f°)", radius, theta, Math.toDegrees(theta));
    }
}