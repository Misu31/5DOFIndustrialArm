package org.firstinspires.ftc.teamcode.Olimpiada2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto oncs 4", group = "Auto 1.9 tdi")
public class autoONCSV2 extends OpMode {


    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose   = new Pose(56,  8, Math.toRadians( 90));
    private final Pose scorePose   = new Pose(56, 36, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(20, 36, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(58.617, 61.953, Math.toRadians(45));
    private final Pose pickup3Pose = new Pose(21.813, 18.518, Math.toRadians(180));

    private Path scorePreload;
    private PathChain inter, grabPickup1, scorePickup1;


    DcMotorEx baseMotor, shoulderMotor, elbowMotor;
    Servo     pitchServo, claw, rollServo, claw2;


    static final double L1 = 200.0;
    static final double L2 = 200.0;
    public double degperpos=0.00138889;

    static final double TICKS_BASE     = 1425.1 * 3;
    static final double TICKS_SHOULDER = 1425.1 * 5;
    static final double TICKS_ELBOW    =  1425.1 * 3;

    static final double BASE_HALF    = 180.0;
    static final double SHOULDER_MIN =   0.0;
    static final double SHOULDER_MAX = 180.0;
    static final double ELBOW_HALF   = 135.0;
    static final double PITCH_HALF   =  90.0;

    static final int ELBOW_AUTO =  0;
    static final int ELBOW_UP   = +1;
    static final int ELBOW_DOWN = -1;

    static final double SERVO_INITIAL = 0.47;
    static final double SERVO_LIM_SUP = 0.6;
    static final double SERVO_LIM_INF = 0.35;

    static final double ROLL_INITIAL  = 0.513;
    static final double ROLL_MIN      = 0.0;
    static final double ROLL_MAX      = 1.0;


    static final double BASE_POWER     = 0.4282051;
    static final double SHOULDER_POWER = 0.5;
    static final double ELBOW_POWER    = 0.4282051;


    static final double CLAW_OPEN   = 0.0;
    static final double CLAW_CLOSE  = 0.4;
    static final double CLAW2_OPEN  = 0.4;
    static final double CLAW2_CLOSE = 0.08;


    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        inter = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup3Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup3Pose.getHeading())
                .build();
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
                    case 0: //ridica bratul si dupa 3 secunde merge la pozitie
                        // This runs immediately at startup while the robot stays in place
                        moveArm(200, 0, 200, 90, ELBOW_DOWN);
                        openClaws();
                        // Wait 3 seconds for the arm to finish before driving
                        if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                            follower.followPath(scorePreload);
                            pathTimer.resetTimer();
                            setPathState(1);///1
                        }
                        break;

                    case 1: //muta bratul deasupra cubului
                        // Wait until the robot completely finishes traveling to point B
                        if (!follower.isBusy()) {
                            // ─── MOVE ARM: MOVE 2 CUBES OUT OF THE WAY ───
                            moveArm(150, 230, 169, 45, ELBOW_AUTO);
                            setPathState(2); // Transition to the arm-waiting state
                        }
                        break;

                    case 2: //muta bratul sa ia cubul
                        if(pathTimer.getElapsedTimeSeconds()>3){
                            closeClaws();
                            pathTimer.resetTimer();
                            setPathState(3);
                        }
                        break;
                    case 3: //muta bratul intro pozitie intermediara
                        if(pathTimer.getElapsedTimeSeconds()>3){
                            moveArm(100, -230, 160, 0,ELBOW_AUTO);
                            pathTimer.resetTimer();
                            setPathState(4);
                        }
                         break;
                    case 4: // arrived at drop position → open claws first
                        if(pathTimer.getElapsedTimeSeconds()>4){
                            openClaws();
                            pathTimer.resetTimer();
                            setPathState(5);
                        }
                        break;

                    case 5: // wait for claw to physically open, THEN move arm away
                        if(pathTimer.getElapsedTimeSeconds()>4.0){
                            moveArm(80, 230, 160, 45, ELBOW_AUTO);
                            pathTimer.resetTimer();
                            setPathState(6);
                        }
                        break;

                    case 6: // close claws on second cube
                        if(pathTimer.getElapsedTimeSeconds()>4){
                            closeClaws();
                            pathTimer.resetTimer();
                            setPathState(61); // lift first
                        }
                        break;

                    case 61: // wait for claw to close, then lift slightly to clear the ground
                        if(pathTimer.getElapsedTimeSeconds()>3.0){
                            moveArm(80, 230, 210, 45, ELBOW_AUTO); // same x/y, z bumped up from 160
                            pathTimer.resetTimer();
                            setPathState(62);
                        }
                        break;
                    case 62:
                        if(pathTimer.getElapsedTimeSeconds()>3){
                            moveArm(150, -230, 210, 45, ELBOW_AUTO);
                            pathTimer.resetTimer();
                            setPathState(7);

                        }
                        break;
                    case 7: // now safe to swing to the drop position
                        if(pathTimer.getElapsedTimeSeconds()>4){
                            moveArm(150, -230, 160, 45, ELBOW_AUTO);
                            pathTimer.resetTimer();
                            setPathState(8);
                        }
                        break;
                    case 8:
                        if(pathTimer.getElapsedTimeSeconds()>3){
                            openClaws();
                            pathTimer.resetTimer();
                            setPathState(9);
                        }
                        break;
                    case 9:
                        moveArm(200, 0, 200, 0, ELBOW_DOWN);
                        if(!follower.isBusy()){
                            follower.followPath(inter, true);
                            setPathState(10);
                        }
                        break;
                    case 10:
                        if(pathTimer.getElapsedTimeSeconds()>3){
                        moveArm(320, 0, 144, 0, ELBOW_AUTO);
                        pathTimer.resetTimer();
                        setPathState(11);
                        }
                        break;
                    case 11: // arm is at pickup position → close claws
                        if (pathTimer.getElapsedTimeSeconds() > 3) {
                            closeClaws();
                            pathTimer.resetTimer();
                            setPathState(67);
                        }
                        break;

                    case 67: // wait for claw servo to physically close (~0.5–1s is enough)
                        if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                            moveArm(200, -150, 230, 0, ELBOW_UP); // move AFTER claw is shut
                            pathTimer.resetTimer();
                            setPathState(12);
                        }
                        break; // ← THIS WAS MISSING — caused the fall-through

                    case 12: // wait for arm to reach scoring position, THEN open claws
                        if (pathTimer.getElapsedTimeSeconds() > 3) {
                            openClaws(); // open only AFTER arm has stopped
                            pathTimer.resetTimer();
                            setPathState(13);
                        }
                        break;

                    case 13: // wait for claw servo to physically open, then continue
                        if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                            pathTimer.resetTimer();
                            setPathState(14);
                        }
                        break;
                    case 14:
                        if(!follower.isBusy()){
                            follower.followPath(grabPickup1, true);
                            moveArm(150, 0 ,0, -90, ELBOW_UP);
                            setPathState(15);
                        }
                        break;
                    case 15:
                        if(pathTimer.getElapsedTimeSeconds()>3){
                            moveArm(340, 0 ,-35, -60, ELBOW_UP);
                            pathTimer.resetTimer();
                            setPathState(16);
                        }
                        break;
                    case 16:
                        if(pathTimer.getElapsedTimeSeconds()>3){
                            closeClaws();
                            pathTimer.resetTimer();
                            setPathState(17);
                        }
                        break;
                    case 17:
                        if(pathTimer.getElapsedTimeSeconds()>3){
                            moveArm(200,0,200, 0, ELBOW_UP);
                            pathTimer.resetTimer();
                            setPathState(20);
                        }
                        break;
                    case 20:
                        if(pathTimer.getElapsedTimeSeconds()>3){
                            moveArm(200,0,200,-90,ELBOW_UP);
                            setPathState(18);
                            pathTimer.resetTimer();
                        }
                    case 18:
                        if(pathTimer.getElapsedTimeSeconds()>3){
                            follower.followPath(scorePickup1, true);
                            setPathState(19);
                        }
                        break;
                    /*case 6:
                        // Robot stays perfectly still here because no new path has been given yet
                        // Wait 3 seconds for the heavy gear reductions to finish moving the arm
                        moveArm(12,12,12,12,ELBOW_AUTO);
                        if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                            follower.followPath(inter); // Start path B -> C
                            pathTimer.resetTimer();
                            setPathState(7);
                        }
                        break;
                    case -7:
                        if(pathTimer.getElapsedTimeSeconds()>4){
                            moveArm(150, 230, 169, 0, ELBOW_AUTO);
                            pathTimer.resetTimer();
                            setPathState(8);
                        }
                        break;
                    case -8:
                        if(pathTimer.getElapsedTimeSeconds()>1){
                            closeClaws();
                            pathTimer.resetTimer();
                            setPathState(9);
                        }
                        break;
                    case -9:
                        if(pathTimer.getElapsedTimeSeconds()>4){
                            moveArm(200, 0, 200, 90, ELBOW_AUTO);
                            pathTimer.resetTimer();
                            setPathState(10);
                        }
                        break;

                    case 7://al doilea cub same bafoonery
                        // Wait until the robot completely finishes traveling to point C
                        if (pathTimer.getElapsedTimeSeconds()>4) {
                            // ─── MOVE ARM: GRAB 2 CUBES FROM SHELF ───
                            moveArm(230, 150, 169, 0, ELBOW_AUTO);

                            setPathState(8); // Transition to the arm-waiting state
                        }
                        break;

                    case 8:
                        // Robot stays still while the arm interacts with the shelf
                        if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                            follower.followPath(grabPickup1); // Start path C -> D
                            setPathState(9);
                        }
                        break;

                    case 10:
                        // Wait until the robot completely finishes traveling to point D
                        if (!follower.isBusy()) {
                            // ─── MOVE ARM: GRAB WATER BOTTLE ───
                            moveArm(100, 170, 169, 0, ELBOW_AUTO);

                            setPathState(11); // Transition to the arm-waiting state
                        }
                        break;

                    case 11:
                        // Robot stays still while the arm secures the water bottle
                        if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                            follower.followPath(scorePickup1); // Start path D -> E (Heading to you)
                            setPathState(12);
                        }
                        break;

                    case 12:
                        // Wait until the robot arrives right in front of you (Point E)
                        if (!follower.isBusy()) {
                            // ─── MOVE ARM: HAND OVER WATER BOTTLE ───
                            // Adjust these coordinates to whatever your delivery/handover position is!
                            moveArm(200, 0, 200, 90, ELBOW_AUTO);

                            setPathState(-1); // Autonomous complete!
                        }
                        break;
                    */
                    default:
                        // Do nothing, autonomous is finished
                        break;

            /*
            case 0:
                moveArm(200,0,200,90,ELBOW_DOWN);
                if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                    pathTimer.resetTimer();
                    setPathState(1);
                }
                break;


            case 1:
                follower.followPath(scorePreload);
                setPathState(2);
                break;

            case 2:
                if (!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3) { //pozitia de mutat cuburi dintr-un loc in altul
                    moveArm(150, 230, 169, 0, ELBOW_AUTO);
                    pathTimer.resetTimer();
                    follower.followPath(inter);
                    setPathState(3);
                }
                break;


            case 3:
                if (!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3) {//pozitia in care ia de pe raft
                    moveArm(230, 150, 169, 0, ELBOW_AUTO);
                    pathTimer.resetTimer();
                    follower.followPath(grabPickup1);
                    setPathState(4);
                }
                break;


            case 4:
                if (!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3) {//pozitia de luat sticla
                    moveArm(100, 170, 169, 0, ELBOW_AUTO);
                    pathTimer.resetTimer();
                    follower.followPath(scorePickup1);//urmatorul case imi va da sticla
                    setPathState(-1);
                }
                break; */
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void openClaws(){
        claw.setPosition(CLAW_OPEN);
        claw2.setPosition(CLAW2_OPEN);
    }
    public void closeClaws(){
        claw.setPosition(CLAW_CLOSE);
        claw2.setPosition(CLAW2_CLOSE);
    }


    @Override
    public void init() {
        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        baseMotor     = hardwareMap.get(DcMotorEx.class, "theta1");
        shoulderMotor = hardwareMap.get(DcMotorEx.class, "theta2");
        elbowMotor    = hardwareMap.get(DcMotorEx.class, "theta3");

        baseMotor    .setDirection(DcMotorSimple.Direction.REVERSE);
        shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor   .setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{ baseMotor, shoulderMotor, elbowMotor }) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        pitchServo = hardwareMap.get(Servo.class, "theta4");
        claw       = hardwareMap.get(Servo.class, "claw");
        claw2       = hardwareMap.get(Servo.class, "claw2");
        rollServo  = hardwareMap.get(Servo.class, "roll");

        pitchServo.setPosition(SERVO_INITIAL);
        claw2      .setPosition(CLAW2_OPEN);
        claw      .setPosition(CLAW_OPEN);
        rollServo .setPosition(ROLL_INITIAL);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x",          follower.getPose().getX());
        telemetry.addData("y",          follower.getPose().getY());
        telemetry.addData("heading",    follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {}



    void moveArm(double x, double y, double z, double pitchDeg, int elbowPref) {

        double[] sol = solveIK(x, y, z, pitchDeg, elbowPref);
        if (sol == null) return;

        int baseTicks     = degreesToTicks(sol[0], TICKS_BASE);
        int shoulderTicks = degreesToTicks(sol[1], TICKS_SHOULDER);
        int elbowTicks    = degreesToTicks(sol[2], TICKS_ELBOW);

        baseMotor    .setTargetPosition(baseTicks);
        shoulderMotor.setTargetPosition(shoulderTicks);
        elbowMotor   .setTargetPosition(elbowTicks);

        baseMotor    .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor   .setMode(DcMotor.RunMode.RUN_TO_POSITION);

        baseMotor    .setPower(BASE_POWER);
        shoulderMotor.setPower(SHOULDER_POWER);
        elbowMotor   .setPower(ELBOW_POWER);

        pitchServo.setPosition(pitchDegToServoPos(sol[3]));
    }


    double pitchDegToServoPos(double angleDeg) {
        /*angleDeg = Math.max(-SERVO_MAX_DEG, Math.min(SERVO_MAX_DEG, angleDeg));
        double scale = (angleDeg >= 0)
                ? (SERVO_LIM_SUP - SERVO_INITIAL) / SERVO_MAX_DEG   // +side: 0.05 / 90
                : (SERVO_INITIAL - SERVO_LIM_INF) / SERVO_MAX_DEG;  // -side: 0.05 / 90

        double pos = SERVO_INITIAL + angleDeg * scale;

        return Math.max(SERVO_LIM_INF, Math.min(SERVO_LIM_SUP, pos));
        */
        return SERVO_INITIAL-angleDeg*degperpos;
    }



    double scoreSolution(double baseDeg, double shoulderDeg,
                         double elbowDeg, double pitchRelDeg) {

        if (Math.abs(baseDeg)     > BASE_HALF)  return Double.MAX_VALUE;
        if (Math.abs(elbowDeg)    > ELBOW_HALF) return Double.MAX_VALUE;
        if (Math.abs(pitchRelDeg) > PITCH_HALF) return Double.MAX_VALUE;

        if (shoulderDeg < SHOULDER_MIN || shoulderDeg > SHOULDER_MAX) return Double.MAX_VALUE;

        return Math.abs(shoulderDeg - 90.0) + Math.abs(elbowDeg) + Math.abs(pitchRelDeg);
    }


    double[] solveIK(double x, double y, double z, double pitchDeg, int elbowPref) {

        double baseDeg = Math.toDegrees(Math.atan2(y, x));
        double r       = Math.hypot(x, y);

        double cosElbow = (r*r + z*z - L1*L1 - L2*L2) / (2.0 * L1 * L2);
        if (cosElbow > 1.0 || cosElbow < -1.0) {
            telemetry.addData("IK FAIL", "Out of reach  r=%.0f  z=%.0f", r, z);
            telemetry.update();
            return null;
        }

        double elbowMag = Math.acos(cosElbow);

        int[] signsToTry = (elbowPref == ELBOW_DOWN)
                ? new int[]{ -1, +1 }
                : new int[]{ +1, -1 };

        double   bestScore    = Double.MAX_VALUE;
        double[] bestSolution = null;

        for (int sign : signsToTry) {
            double elbowRad    = sign * elbowMag;
            double elbowDeg    = Math.toDegrees(elbowRad);

            double shoulderRad = Math.atan2(z, r)
                    - Math.atan2(L2 * Math.sin(elbowRad),
                    L1 + L2 * Math.cos(elbowRad));
            double shoulderDeg = Math.toDegrees(shoulderRad);

            double pitchRelDeg = shoulderDeg + elbowDeg - pitchDeg;

            double score = scoreSolution(baseDeg, shoulderDeg, elbowDeg, pitchRelDeg);

            telemetry.addData("IK " + (sign > 0 ? "UP  " : "DOWN"),
                    score == Double.MAX_VALUE
                            ? String.format("INVALID  sh=%.1f° el=%.1f° pt=%.1f°",
                            shoulderDeg, elbowDeg, pitchRelDeg)
                            : String.format("score=%.1f  sh=%.1f° el=%.1f° pt=%.1f°",
                            score, shoulderDeg, elbowDeg, pitchRelDeg));

            if (elbowPref != ELBOW_AUTO && score < Double.MAX_VALUE && bestSolution == null) {
                bestScore    = score;
                bestSolution = new double[]{ baseDeg, shoulderDeg, elbowDeg, pitchRelDeg, sign };
                continue;
            }

            if (score < bestScore) {
                bestScore    = score;
                bestSolution = new double[]{ baseDeg, shoulderDeg, elbowDeg, pitchRelDeg, sign };
            }
        }

        if (bestSolution == null || bestScore == Double.MAX_VALUE) {
            telemetry.addData("IK FAIL", "No valid solution found");
            telemetry.update();
            return null;
        }

        telemetry.addData("IK chose", bestSolution[4] > 0 ? "ELBOW-UP" : "ELBOW-DOWN");
        return bestSolution;
    }


    int degreesToTicks(double degrees, double ticksPerRev) {
        return (int) Math.round(degrees / 360.0 * ticksPerRev);
    }
}