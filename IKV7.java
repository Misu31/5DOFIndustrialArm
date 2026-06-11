package org.firstinspires.ftc.teamcode.Olimpiada2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "IKV7", group = "Arm")
public class IKV7 extends LinearOpMode {


    static final String BASE_MOTOR     = "theta1";
    static final String SHOULDER_MOTOR = "theta2";
    static final String ELBOW_MOTOR    = "theta3";
    static final String PITCH_SERVO    = "theta4";
    static final String CLAW = "claw";
    static final String ROLL_CLAW = "roll";


    static final double L1 = 200.0;
    static final double L2 = 200.0;

    static final double TICKS_BASE     = 1425.1 * 3;   // 4275.3
    static final double TICKS_SHOULDER = 1425.1 * 5;   // 7125.5
    static final double TICKS_ELBOW    =  1425.1 * 3;   // 1613.1


    static final double BASE_HALF      = 180.0;
    static final double SHOULDER_MIN   =   0.0;
    static final double SHOULDER_MAX   = 180.0;
    static final double ELBOW_HALF     = 135.0;
    static final double PITCH_HALF     =  90.0;


    static final int ELBOW_AUTO =  0;
    static final int ELBOW_UP   = +1;
    static final int ELBOW_DOWN = -1;



    static final double SERVO_INITIAL = 0.475;
    public double degperpos=0.00138889;
    static final double SERVO_LIM_SUP = 0.44;
    static final double SERVO_LIM_INF = 0.34;
    static final double SERVO_MAX_DEG = 90.0;
    static final double BASE_POWER     = 0.50;
    static final double SHOULDER_POWER = 0.8;
    static final double ELBOW_POWER    = 0.45;
    static final int    POSITION_TOLERANCE = 15;
    static final long   MOVE_TIMEOUT_MS    = 5000;


    DcMotorEx baseMotor, shoulderMotor, elbowMotor;
    Servo     pitchServo, claw, rollServo, claw2;


    @Override
    public void runOpMode() {

        initHardware();
        telemetry.addLine("Ready. Arm at init position.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            /*moveArm(200, 150, 155, -90, ELBOW_AUTO);
            sleep(5000);
            moveArm(200, 150, 155, -45, ELBOW_AUTO);
            sleep(5000);
            moveArm(200, 150, 155, 0, ELBOW_AUTO);
            sleep(5000);
            moveArm(200, 150, 155, 45, ELBOW_AUTO);
            sleep(5000);
            moveArm(200, 150, 155, 90, ELBOW_AUTO);
            sleep(10000);*/

            ///-------------------------

            moveArm(200, 0,155, -45, ELBOW_AUTO); //aparent -45 e 0... ok bizar... pozitie de inceput
            sleep(800);
            moveArm(180,180,155,-45,ELBOW_AUTO); //sa stea deasupra cubului
            sleep(800);
            moveArm(180, 180, 100, 0, ELBOW_AUTO);//pozitie deasupra cubului si cu grheara in jos
            sleep(800);
            moveArm(180,180,100,0,ELBOW_AUTO);
            sleep(800);
            //claw.setPosition(ClawClose);
            clawClose();
            sleep(1000);
            moveArm(200, 0,155, -45, ELBOW_AUTO); //aparent -45 e 0... ok bizar... pozitie de inceput
            sleep(800);
            moveArm(200,0,120,0, ELBOW_AUTO);
            sleep(800);
            //claw.setPosition(ClawOpen);
            clawOpen();
            sleep(500);

            ///-------------------------

            /*moveArm(180, 180, 155, 90, ELBOW_AUTO);//pozitie deasupra cubului si cu grheara in jos
            sleep(800);
            claw.setPosition(ClawClose); // inchis gheara
            sleep(500);
            moveArm(180,180,150, 90, ELBOW_AUTO); //ridicat cub de jos
            sleep(800);
            moveArm(200, 0, 155, -45, ELBOW_AUTO); //dus la pozitie
            sleep(800);
            moveArm(200, 0, 120, 90, ELBOW_AUTO);
            sleep(500);
            claw.setPosition(ClawOpen);//lasa cubul
            sleep(500);
            moveArm(200,0,155, -45, ELBOW_AUTO);//inapoi la pozitie de inceput*/
        }
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

        // Base and symmetric joints
        if (Math.abs(baseDeg)     > BASE_HALF)  return Double.MAX_VALUE;
        if (Math.abs(elbowDeg)    > ELBOW_HALF) return Double.MAX_VALUE;
        if (Math.abs(pitchRelDeg) > PITCH_HALF) return Double.MAX_VALUE;

        if (shoulderDeg < SHOULDER_MIN || shoulderDeg > SHOULDER_MAX) return Double.MAX_VALUE;


        double scoreShoulder = Math.abs(shoulderDeg - 90.0);
        double scoreElbow    = Math.abs(elbowDeg);
        double scorePitch    = Math.abs(pitchRelDeg);

        return scoreShoulder + scoreElbow + scorePitch;
    }


    double[] solveIK(double x, double y, double z, double pitchDeg, int elbowPref) {

        double baseDeg = Math.toDegrees(Math.atan2(y, x));

        double r = Math.hypot(x, y);

        double cosElbow = (r*r + z*z - L1*L1 - L2*L2) / (2.0 * L1 * L2);
        if (cosElbow > 1.0 || cosElbow < -1.0) {
            telemetry.addData("IK FAIL", "Out of reach (r=%.0f, z=%.0f)", r, z);
            telemetry.update();
            return null;
        }

        double elbowMag = Math.acos(cosElbow);   //[0°, 180°]

        int[] signsToTry;
        if      (elbowPref == ELBOW_UP)   signsToTry = new int[]{ +1, -1 };
        else if (elbowPref == ELBOW_DOWN) signsToTry = new int[]{ -1, +1 };
        else                               signsToTry = new int[]{ +1, -1 };   // AUTO

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
        telemetry.update();
        return bestSolution;
    }



    int degreesToTicks(double degrees, double ticksPerRev) {
        return (int) Math.round(degrees / 360.0 * ticksPerRev);
    }



    void moveArm(double x, double y, double z, double pitchDeg, int elbowPref) {

        double[] sol = solveIK(x, y, z, pitchDeg, elbowPref);
        if (sol == null) return;

        double baseDeg     = sol[0];
        double shoulderDeg = sol[1];
        double elbowDeg    = sol[2];
        double pitchRelDeg = sol[3];

        int baseTicks     = degreesToTicks(baseDeg,     TICKS_BASE);
        int shoulderTicks = degreesToTicks(shoulderDeg, TICKS_SHOULDER);
        int elbowTicks    = degreesToTicks(elbowDeg,    TICKS_ELBOW);

        double pitchPos = pitchDegToServoPos(pitchRelDeg);

        baseMotor    .setTargetPosition(baseTicks);
        shoulderMotor.setTargetPosition(shoulderTicks);
        elbowMotor   .setTargetPosition(elbowTicks);

        baseMotor    .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor   .setMode(DcMotor.RunMode.RUN_TO_POSITION);

        baseMotor    .setPower(BASE_POWER);
        shoulderMotor.setPower(SHOULDER_POWER);
        elbowMotor   .setPower(ELBOW_POWER);

        pitchServo.setPosition(pitchPos);

        telemetry.addData("Target (mm)",  "(%.0f, %.0f, %.0f)  pitch=%.1f°", x, y, z, pitchDeg);
        telemetry.addData("Base",         "%.2f°  →  %d ticks", baseDeg,     baseTicks);
        telemetry.addData("Shoulder",     "%.2f°  →  %d ticks", shoulderDeg, shoulderTicks);
        telemetry.addData("Elbow",        "%.2f°  →  %d ticks", elbowDeg,    elbowTicks);
        telemetry.addData("Pitch",        "%.2f° rel  →  servo %.4f", pitchRelDeg, pitchPos);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        while (!isStopRequested() && timer.milliseconds() < MOVE_TIMEOUT_MS) {
            boolean baseDone     = Math.abs(baseMotor    .getCurrentPosition() - baseTicks)     <= POSITION_TOLERANCE;
            boolean shoulderDone = Math.abs(shoulderMotor.getCurrentPosition() - shoulderTicks) <= POSITION_TOLERANCE;
            boolean elbowDone    = Math.abs(elbowMotor   .getCurrentPosition() - elbowTicks)    <= POSITION_TOLERANCE;
            idle();
            if (baseDone && shoulderDone && elbowDone) break;
        }

        sleep(150);
    }


    void initHardware() {

        baseMotor     = hardwareMap.get(DcMotorEx.class, BASE_MOTOR);
        shoulderMotor = hardwareMap.get(DcMotorEx.class, SHOULDER_MOTOR);
        elbowMotor    = hardwareMap.get(DcMotorEx.class, ELBOW_MOTOR);
        pitchServo    = hardwareMap.get(Servo.class,     PITCH_SERVO);
        claw = hardwareMap.get(Servo.class, CLAW);
        claw2=hardwareMap.get(Servo.class, "claw2");
        rollServo = hardwareMap.get(Servo.class, ROLL_CLAW);

        baseMotor    .setDirection(DcMotorSimple.Direction.REVERSE);
        shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor   .setDirection(DcMotorSimple.Direction.FORWARD);

        baseMotor    .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor   .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        baseMotor    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitchServo.setPosition(SERVO_INITIAL);   // park at 0
        clawOpen();
        //claw.setPosition(ClawOpen);
        rollServo.setPosition(0.513);
    }
    public void clawOpen(){
        claw.setPosition(0);
        claw2.setPosition(0.4);
    }
    public void clawClose(){
        claw.setPosition(0.4);
        claw2.setPosition(0.08);
    }
}