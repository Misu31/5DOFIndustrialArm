package org.firstinspires.ftc.teamcode.Olimpiada2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ArmTeleOp v2", group = "Arm")
public class TeleOpIKv2 extends LinearOpMode {

    static final String BASE_MOTOR     = "theta1";
    static final String SHOULDER_MOTOR = "theta2";
    static final String ELBOW_MOTOR    = "theta3";
    static final String PITCH_SERVO    = "theta4";
    static final String CLAW           = "claw";
    static final String ROLL_CLAW      = "roll";
    static final double L1 = 200.0;
    static final double L2 = 200.0;
    static final double TICKS_BASE     = 1425.1 * 3;
    static final double TICKS_SHOULDER = 1425.1 * 5;
    static final double TICKS_ELBOW    =  1425.1 * 3;


    static final double BASE_HALF    = 180.0;
    static final double SHOULDER_MIN =   0.0;
    static final double SHOULDER_MAX = 180.0;
    static final double ELBOW_HALF   = 135.0;


    static final int ELBOW_AUTO = 0;
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


    static final double REACH_SPEED    = 2.0;
    static final double Z_SPEED        = 2.0;
    static final double BASE_ROT_SPEED = 1.0;
    static final double JOYSTICK_DEAD  = 0.15;
    static final double SERVO_STEP     = 0.02;

    static final double R_MIN = 50.0;
    static final double R_MAX = 370.0;
    static final double Z_MIN = -200.0;
    static final double Z_MAX = 370.0;

    static final double REACH_OUTER_MAX = L1 + L2 - 15.0;
    static final double REACH_INNER_MIN = 30.0;

    static final double INIT_R     = 200.0;
    static final double INIT_BASE  =   0.0;
    static final double INIT_Z     = 155.0;

    DcMotorEx baseMotor, shoulderMotor, elbowMotor;
    DcMotor stangaSus, dreaptaSus, stangaJos, dreaptaJos;
    Servo     pitchServo, claw, rollServo, claw2;


    double currentR         = INIT_R;
    double currentBaseAngle = INIT_BASE;
    double currentZ         = INIT_Z;

    double lastValidR         = INIT_R;
    double lastValidBaseAngle = INIT_BASE;
    double lastValidZ         = INIT_Z;


    double  currentPitchPos = SERVO_INITIAL;
    double  currentRollPos  = ROLL_INITIAL;
    boolean clawOpen        = true;
    boolean ikValid         = true;
    boolean prevDpadUp    = false;
    boolean prevDpadDown  = false;
    boolean prevLBumper   = false;
    boolean prevRBumper   = false;
    boolean prevAButton   = false;
    boolean prevBackBtn   = false;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Ready — press START");
        telemetry.update();
        waitForStart();

        goToInitPose();

        while (opModeIsActive()) {
            double max;

            double axial   = -gamepad2.left_stick_y;
            double lateral =  gamepad2.left_stick_x;
            double yaw     =  gamepad2.right_stick_x;

            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max  , Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            stangaSus.setPower(frontLeftPower/2);
            dreaptaSus.setPower(frontRightPower/2);
            stangaJos.setPower(backLeftPower/2);
            dreaptaJos.setPower(backRightPower/2);

            handleJoysticks();
            handleButtons();
            sendToMotors();
            updateTelemetry();
        }
    }
    void handleJoysticks() {
        double savedR    = currentR;
        double savedZ    = currentZ;
        double savedBase = currentBaseAngle;

        double leftY  = applyDeadband(-gamepad1.left_stick_y, JOYSTICK_DEAD);
        double leftX  = applyDeadband( gamepad1.left_stick_x, JOYSTICK_DEAD);
        double rightY = applyDeadband(-gamepad1.right_stick_y, JOYSTICK_DEAD);

        currentR         = clamp(currentR         + leftY  * REACH_SPEED,    R_MIN,    R_MAX);
        currentBaseAngle = clamp(currentBaseAngle  + leftX  * BASE_ROT_SPEED, -BASE_HALF, BASE_HALF);
        currentZ         = clamp(currentZ          + rightY * Z_SPEED,        Z_MIN,    Z_MAX);

        double dist = Math.hypot(currentR, currentZ);
        if (dist > REACH_OUTER_MAX || dist < REACH_INNER_MIN) {

            currentR = savedR;
            currentZ = savedZ;
        }
    }

    void handleButtons() {


        boolean dpadUp   = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadUp && !prevDpadUp) {
            currentPitchPos = clamp(currentPitchPos + SERVO_STEP, SERVO_LIM_INF, SERVO_LIM_SUP);
            pitchServo.setPosition(currentPitchPos);
        }
        if (dpadDown && !prevDpadDown) {
            currentPitchPos = clamp(currentPitchPos - SERVO_STEP, SERVO_LIM_INF, SERVO_LIM_SUP);
            pitchServo.setPosition(currentPitchPos);
        }
        prevDpadUp   = dpadUp;
        prevDpadDown = dpadDown;


        boolean lBumper = gamepad1.left_bumper;
        boolean rBumper = gamepad1.right_bumper;
        if (lBumper && !prevLBumper) {
            currentRollPos = clamp(currentRollPos - SERVO_STEP, ROLL_MIN, ROLL_MAX);
            rollServo.setPosition(currentRollPos);
        }
        if (rBumper && !prevRBumper) {
            currentRollPos = clamp(currentRollPos + SERVO_STEP, ROLL_MIN, ROLL_MAX);
            rollServo.setPosition(currentRollPos);
        }
        prevLBumper = lBumper;
        prevRBumper = rBumper;


        boolean aButton = gamepad1.x;
        if (aButton && !prevAButton) {
            clawOpen = !clawOpen;
            claw .setPosition(clawOpen ? CLAW_OPEN  : CLAW_CLOSE);
            claw2.setPosition(clawOpen ? CLAW2_OPEN : CLAW2_CLOSE);
        }
        prevAButton = aButton;

        boolean backBtn = gamepad1.back;
        if (backBtn && !prevBackBtn) {
            goToInitPose();
        }
        prevBackBtn = backBtn;

        if (gamepad1.dpad_right) {
            zona0();
        }
    }

    void zona0() {
        currentR         = INIT_R;
        currentBaseAngle = INIT_BASE;
        currentZ         = INIT_Z;
        lastValidR         = INIT_R;
        lastValidBaseAngle = INIT_BASE;
        lastValidZ         = INIT_Z;

        pitchServo.setPosition(SERVO_INITIAL);
        rollServo .setPosition(ROLL_INITIAL);
        currentPitchPos = SERVO_INITIAL;
        currentRollPos  = ROLL_INITIAL;

        // sendToMotors()
    }


    void goToInitPose() {
        currentR         = INIT_R;
        currentBaseAngle = INIT_BASE;
        currentZ         = INIT_Z;
        lastValidR         = INIT_R;
        lastValidBaseAngle = INIT_BASE;
        lastValidZ         = INIT_Z;

        pitchServo.setPosition(SERVO_INITIAL);
        rollServo .setPosition(ROLL_INITIAL);
        currentPitchPos = SERVO_INITIAL;
        currentRollPos  = ROLL_INITIAL;

        sendToMotors();

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < 3000) {
            telemetry.addLine("Moving to init pose…");
            telemetry.update();
            idle();
        }
    }

    void sendToMotors() {
        double rad = Math.toRadians(currentBaseAngle);
        double x   = currentR * Math.cos(rad);
        double y   = currentR * Math.sin(rad);

        double[] sol = solveIK(x, y, currentZ, ELBOW_AUTO);

        if (sol == null) {
            ikValid          = false;
            currentR         = lastValidR;
            currentBaseAngle = lastValidBaseAngle;
            currentZ         = lastValidZ;
            return;
        }
        ikValid            = true;
        lastValidR         = currentR;
        lastValidBaseAngle = currentBaseAngle;
        lastValidZ         = currentZ;

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
    }

    double[] solveIK(double x, double y, double z, int elbowPref) {
        double baseDeg = Math.toDegrees(Math.atan2(y, x));
        double r       = Math.hypot(x, y);

        double cosElbow = (r * r + z * z - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
        if (cosElbow > 1.0 || cosElbow < -1.0) return null;

        double elbowMag = Math.acos(cosElbow);

        int[] signsToTry = (elbowPref == ELBOW_UP)  ? new int[]{ +1, -1 }
                : (elbowPref == ELBOW_DOWN) ? new int[]{ -1, +1 }
                :                             new int[]{ +1, -1 };

        double   bestScore    = Double.MAX_VALUE;
        double[] bestSolution = null;

        for (int sign : signsToTry) {
            double elbowRad = sign * elbowMag;
            double elbowDeg = Math.toDegrees(elbowRad);

            double shoulderRad = Math.atan2(z, r)
                    - Math.atan2(L2 * Math.sin(elbowRad),
                    L1 + L2 * Math.cos(elbowRad));
            double shoulderDeg = Math.toDegrees(shoulderRad);

            if (Math.abs(baseDeg)  > BASE_HALF)  continue;
            if (Math.abs(elbowDeg) > ELBOW_HALF) continue;
            if (shoulderDeg < SHOULDER_MIN || shoulderDeg > SHOULDER_MAX) continue;

            double score = Math.abs(shoulderDeg - 90.0) + Math.abs(elbowDeg);

            if (elbowPref != ELBOW_AUTO && bestSolution == null) {
                bestSolution = new double[]{ baseDeg, shoulderDeg, elbowDeg };
                bestScore    = score;
                continue;
            }
            if (score < bestScore) {
                bestScore    = score;
                bestSolution = new double[]{ baseDeg, shoulderDeg, elbowDeg };
            }
        }
        return bestSolution;
    }

    int    degreesToTicks(double deg, double tpr) { return (int) Math.round(deg / 360.0 * tpr); }
    double clamp(double v, double lo, double hi)  { return Math.max(lo, Math.min(hi, v)); }

    double applyDeadband(double value, double dead) {
        if (Math.abs(value) < dead) return 0.0;
        return Math.signum(value) * (Math.abs(value) - dead) / (1.0 - dead);
    }

    void updateTelemetry() {
        double rad = Math.toRadians(currentBaseAngle);
        telemetry.addData("IK status",    ikValid ? "OK" : "!! REVERTED — was out of reach");
        telemetry.addData("XYZ target",   "x=%.0f  y=%.0f  z=%.0f",
                currentR * Math.cos(rad), currentR * Math.sin(rad), currentZ);
        telemetry.addData("Reach / Angle","r=%.0f mm  base=%.1f°", currentR, currentBaseAngle);
        telemetry.addData("Dist to shldr","%.0f mm  (max %.0f)", Math.hypot(currentR, currentZ), REACH_OUTER_MAX);
        telemetry.addData("Pitch servo",  "%.3f  (dpad ↑↓)", currentPitchPos);
        telemetry.addData("Roll servo",   "%.3f  (LB/RB)",   currentRollPos);
        telemetry.addData("Claw",         clawOpen ? "OPEN" : "CLOSED");
        telemetry.addLine("── raw joint readings ──────────────────────");
        telemetry.addData("theta1_deg",   currentBaseAngle);
        telemetry.addData("theta2_deg",   shoulderMotor.getCurrentPosition() / 7125.5 * 360.0);
        telemetry.addData("theta3_deg",   elbowMotor.getCurrentPosition()    / 1613.1 * 360.0);
        telemetry.addData("theta4_pos",   currentPitchPos);
        telemetry.addData("r_mm",         currentR);
        telemetry.addData("z_mm",         currentZ);
        telemetry.update();
    }

    void initHardware() {
        baseMotor     = hardwareMap.get(DcMotorEx.class, BASE_MOTOR);
        shoulderMotor = hardwareMap.get(DcMotorEx.class, SHOULDER_MOTOR);
        elbowMotor    = hardwareMap.get(DcMotorEx.class, ELBOW_MOTOR);
        pitchServo    = hardwareMap.get(Servo.class,     PITCH_SERVO);
        claw          = hardwareMap.get(Servo.class,     CLAW);
        rollServo     = hardwareMap.get(Servo.class,     ROLL_CLAW);
        claw2         = hardwareMap.get(Servo.class,     "claw2");
        stangaSus=hardwareMap.get(DcMotor.class, "stangaSus");
        stangaJos=hardwareMap.get(DcMotor.class, "stangaJos");
        dreaptaJos=hardwareMap.get(DcMotor.class, "dreaptaJos");
        dreaptaSus=hardwareMap.get(DcMotor.class, "dreaptaSus");

        stangaSus.setDirection(DcMotorSimple.Direction.REVERSE);
        dreaptaJos.setDirection(DcMotorSimple.Direction.REVERSE);
        baseMotor    .setDirection(DcMotorSimple.Direction.REVERSE);
        shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor   .setDirection(DcMotorSimple.Direction.FORWARD);

        baseMotor    .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor   .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        baseMotor    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitchServo.setPosition(SERVO_INITIAL);
        claw      .setPosition(CLAW_OPEN);
        rollServo .setPosition(ROLL_INITIAL);

        currentPitchPos = SERVO_INITIAL;
        currentRollPos  = ROLL_INITIAL;
        clawOpen        = true;
    }
}