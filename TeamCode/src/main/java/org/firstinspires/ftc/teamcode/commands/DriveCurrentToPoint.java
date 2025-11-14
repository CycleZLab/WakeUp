package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

public class DriveCurrentToPoint extends CommandBase {
    private FollowerSubsystem follower;
    private Pose targetPos, midPos1, midPos2, midPos3;
    private int numberOfPoint;
    private boolean holdEnd = true;
    private double breaking = 1.0;

    public DriveCurrentToPoint(FollowerSubsystem follower, Pose mid1, Pose mid2, Pose mid3, Pose end) {
        targetPos = new Pose(end.getX(), end.getY(), end.getHeading());
        midPos1 = new Pose(mid1.getX(), mid1.getY());
        midPos2 = new Pose(mid2.getX(), mid2.getY());
        midPos3 = new Pose(mid3.getX(), mid3.getY());
        this.follower = follower;
        numberOfPoint = 5;
    }

    public DriveCurrentToPoint(FollowerSubsystem follower, Pose mid1, Pose mid2, Pose end) {
        targetPos = new Pose(end.getX(), end.getY(), end.getHeading());
        midPos1 = new Pose(mid1.getX(), mid1.getY());
        midPos2 = new Pose(mid2.getX(), mid2.getY());
        this.follower = follower;
        numberOfPoint = 4;
    }

    public DriveCurrentToPoint(FollowerSubsystem follower, Pose mid1, Pose end) {
        targetPos = new Pose(end.getX(), end.getY(), end.getHeading());
        midPos1 = new Pose(mid1.getX(), mid1.getY());
        this.follower = follower;
        numberOfPoint = 3;
    }

    public DriveCurrentToPoint(FollowerSubsystem follower, Pose end) {
        targetPos = new Pose(end.getX(), end.getY(), end.getHeading());
        this.follower = follower;
        numberOfPoint = 2;
    }

    public DriveCurrentToPoint setHoldEnd (boolean holdEnd){
        this.holdEnd = holdEnd;
        return this;
    }

    public DriveCurrentToPoint setBreaking (double breakingStrength){
        this.breaking = breakingStrength;
        return this;
    }

    @Override
    public void initialize() {
        this.follower.breakFollowing();
        PathBuilder builder = new PathBuilder(this.follower.follower);
        if (numberOfPoint == 5) {
            builder.addPath(
                            new BezierCurve(
                                    new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                    new Pose(midPos1.getX(), midPos1.getY()),
                                    new Pose(midPos2.getX(), midPos2.getY()),
                                    new Pose(midPos3.getX(), midPos3.getY()),
                                    new Pose(targetPos.getX(), targetPos.getY())
                            )
                    )
                    .setConstantHeadingInterpolation(follower.getPose().getHeading())
//                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPos.getHeading())
                    .setBrakingStrength(this.breaking);
        } else if (numberOfPoint == 4) {
            builder.addPath(
                            new BezierCurve(
                                    new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                    new Pose(midPos1.getX(), midPos1.getY()),
                                    new Pose(midPos2.getX(), midPos2.getY()),
                                    new Pose(targetPos.getX(), targetPos.getY())
                            )
                    )
                    .setConstantHeadingInterpolation(follower.getPose().getHeading())
//                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPos.getHeading())
                    .setBrakingStrength(this.breaking);
        } else if (numberOfPoint == 3) {
            builder.addPath(
                            new BezierCurve(
                                    new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                    new Pose(midPos1.getX(), midPos1.getY()),
                                    new Pose(targetPos.getX(), targetPos.getY())
                            )
                    )
                    .setConstantHeadingInterpolation(follower.getPose().getHeading())
//                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPos.getHeading())
                    .setBrakingStrength(this.breaking);
        } else {
            builder.addPath(
                            new BezierLine(
                                    new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                    new Pose(targetPos.getX(), targetPos.getY())
                            )
                    )
                    .setConstantHeadingInterpolation(follower.getPose().getHeading())
//                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPos.getHeading())
                    .setBrakingStrength(this.breaking);
        }
        this.follower.breakFollowing();
        this.follower.followPath(builder.build(), this.holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy() || follower.isRobotStuck() || follower.follower.atParametricEnd();
    }

    @Override
    public void end(boolean interrupted){
    }
}
