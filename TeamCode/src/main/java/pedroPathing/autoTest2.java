package pedroPathing;


import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "autoTest2", group = "testing")
public class autoTest2 extends OpMode {
    private Follower follower;
    private Telemetry telemetryA;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(135, 80.4, Math.toRadians(180));
    private final Pose firstPosePreOuttake = new Pose(110, 78, Math.toRadians(180));
    private final Pose secondPoseOuttake = new Pose(106.5, 78, Math.toRadians(180));
    private final Pose thirdPosePrePushControlOne = new Pose(131, 117.5, Math.toRadians(0));
    private final Pose thirdPosePrePushControlTwo = new Pose(79, 102.5, Math.toRadians(0));
    private final Pose thirdPosePrePushOne = new Pose(82, 120, Math.toRadians(0));
    private final Pose fourthPosePostPushOne = new Pose(128, 120, Math.toRadians(0));
    private final Pose fifthPosePrePushTwoControlOne = new Pose(81, 108, Math.toRadians(0));
    private final Pose fifthPosePrePushTwo = new Pose(84, 130, Math.toRadians(0));
    private final Pose sixthPosePostPushTwo = new Pose(128, 130, Math.toRadians(0));
    private final Pose seventhPosePrePushThreeControlOne = new Pose(82, 120, Math.toRadians(0));
    private final Pose seventhPosePrePushThree = new Pose(82, 135.5, Math.toRadians(0));
    private final Pose eighthPosePostPushThree = new Pose(128, 135.5, Math.toRadians(0));
    private final Pose ninthPosePreIntakeOne = new Pose(127, 113, Math.toRadians(90));
    private PathChain action6PushTwo, action24Park, action11turn, action9PreIntakeOne, action16Turn, action22PreOuttakeFourth, action23OuttakeFourth, action15IntakeTwo, action21turn, action21PostIntakeThree, action20IntakeThree, action19PreIntakeThree, action18OuttakeThree, action17PreOuttakeThree,action16PostIntakeTwo, action14PreIntakeTwo, action12PreOuttakeTwo, action10IntakeOne, action11PostIntakeOne, action8PushThree, action7CurvedForPushThree, action1PreOuttakeOne, action5CurvedForPushTwo, action2OuttakeOne, action13OuttakeTwo, action3DoubleCurvedPrePush, action4PushOne;

    public void buildPaths() {
        action1PreOuttakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(firstPosePreOuttake)))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPosePreOuttake.getHeading())
                .build();

        action2OuttakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstPosePreOuttake), new Point(secondPoseOuttake)))
                .setLinearHeadingInterpolation(firstPosePreOuttake.getHeading(), secondPoseOuttake.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action3DoubleCurvedPrePush = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondPoseOuttake), new Point(thirdPosePrePushControlOne), new Point(thirdPosePrePushControlTwo), new Point(thirdPosePrePushOne)))
                .setLinearHeadingInterpolation(secondPoseOuttake.getHeading(), thirdPosePrePushOne.getHeading())
                .build();

        action4PushOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdPosePrePushOne), new Point(fourthPosePostPushOne)))
                .setLinearHeadingInterpolation(thirdPosePrePushOne.getHeading(), fourthPosePostPushOne.getHeading())
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        action5CurvedForPushTwo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(fourthPosePostPushOne), new Point(fifthPosePrePushTwoControlOne), new Point(fifthPosePrePushTwo)))
                .setLinearHeadingInterpolation(fourthPosePostPushOne.getHeading(), fifthPosePrePushTwo.getHeading())
                .build();

        action6PushTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthPosePrePushTwo), new Point(sixthPosePostPushTwo)))
                .setLinearHeadingInterpolation(fifthPosePrePushTwo.getHeading(), sixthPosePostPushTwo.getHeading())
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        action9PreIntakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixthPosePostPushTwo), new Point(ninthPosePreIntakeOne)))
                .setLinearHeadingInterpolation(sixthPosePostPushTwo.getHeading(), ninthPosePreIntakeOne.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(action1PreOuttakeOne, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(action2OuttakeOne, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.holdPoint(secondPoseOuttake);
                    sleep(450);
                    follower.setMaxPower(1);
                    follower.followPath(action3DoubleCurvedPrePush, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(action4PushOne, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(action5CurvedForPushTwo, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(action6PushTwo, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(action9PreIntakeOne, true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        sillyTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {
        opmodeTimer.resetTimer();
        sillyTimer.resetTimer();
    }

    @Override
    public void start() {
        sillyTimer.resetTimer();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}