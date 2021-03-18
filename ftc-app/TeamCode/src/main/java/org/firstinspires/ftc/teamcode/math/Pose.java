package org.firstinspires.ftc.teamcode.math;

public class Pose {

    public double x;
    public double y;
    public double r;

    public Pose(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(Pose p) {
        this(p.x, p.y, p.r);
    }

    public Pose subtract(Pose p) {
        return new Pose(this.x-p.x, this.y-p.y, this.r-p.r);
    }

    public String toString() {
        return String.format("\t%s\t%s\t%s", LCHSMath.round(x, 3), LCHSMath.round(y, 3), LCHSMath.round(r, 3));
    }

    public String toString(String separator) {
        return LCHSMath.round(x, 3) + separator + LCHSMath.round(y, 3) + separator + LCHSMath.round(r, 3);
    }


}
