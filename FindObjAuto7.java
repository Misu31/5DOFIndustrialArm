package org.firstinspires.ftc.teamcode.Olimpiada2;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@Autonomous(name = "FindObjAuto7", group = "Arm")
public class FindObjAuto7 extends LinearOpMode {

    static final String BASE_MOTOR = "theta1";
    static final String SHOULDER_MOTOR = "theta2";
    static final String ELBOW_MOTOR = "theta3";
    static final String PITCH_SERVO = "theta4";
    static final String CLAW = "claw";
    static final String CLAW2 = "claw2";
    static final String ROLL_CLAW = "roll";
    static final String LIMELIGHT = "limelight";

    static final double L1 = 200.0;
    static final double L2 = 200.0;

    static final double TICKS_BASE = 1425.1 * 3.0;
    static final double TICKS_SHOULDER = 1425.1 * 5.0;
    static final double TICKS_ELBOW = 1425.1 * 3.0;

    static final double BASE_HALF = 60.0;
    static final double SHOULDER_MIN = 0.0;
    static final double SHOULDER_MAX = 180.0;
    static final double ELBOW_HALF = 135.0;
    static final double PITCH_HALF = 90.0;

    static final int ELBOW_AUTO = 0;
    static final int ELBOW_UP = 1;
    static final int ELBOW_DOWN = -1;

    static final int APRILTAG_PIPELINE = 3;
    static final int TARGET_TAG_ID = -1;
    static final double LIMELIGHT_HEIGHT_MM = 335.0;
    static final double CAMERA_DOWN_ANGLE_DEG = 30.0;
    static final double CLAW_APPROACH_OFFSET_MM = -60.0;
    static final double END_EFFECTOR_BACKOFF_MM = 0.0;

    static final double SCAN_STEP_DEG = 1.0;
    static final double SCAN_POWER = 0.30;
    static final double CENTER_GAIN = 0.10;
    static final double CENTER_TOLERANCE_DEG = 1.5;
    static final long CENTER_TIMEOUT_MS = 3500;
    static final long STABILIZE_MS = 2000;
    static final int MIN_STABLE_SAMPLES = 8;

    static final double SCAN_PITCH_START_DEG = -45.0;
    static final double SCAN_PITCH_END_DEG = -90.0;
    static final double SCAN_PITCH_STEP_DEG = -5.0;
    static final int SCAN_SWEEPS_PER_PITCH = 3;
    static final long SCAN_PITCH_SETTLE_MS = 150;

    static final double TARGET_Z_MM = 0.0;
    static final double MIN_TARGET_RADIUS_MM = 225.0;
    static final double MAX_TARGET_RADIUS_MM = 480.0;

    static final double IDLE_X = 200.0;
    static final double IDLE_Y = 0.0;
    static final double IDLE_Z = 200.0;
    static final double IDLE_PITCH_DEG = -45.0;

    static final double SERVO_INITIAL = 0.475;
    static final double DEG_PER_POS = 0.00138889;
    static final double ROLL_CENTER = 0.513;

    static final double BASE_POWER = 0.4282051;
    static final double SHOULDER_POWER = 0.50;
    static final double ELBOW_POWER = 0.4282051;
    static final int POSITION_TOLERANCE = 15;
    static final long MOVE_TIMEOUT_MS = 5000;

    DcMotorEx baseMotor;
    DcMotorEx shoulderMotor;
    DcMotorEx elbowMotor;
    Servo pitchServo;
    Servo claw;
    Servo claw2;
    Servo rollServo;
    Limelight3A limelight;
    Servo led;

    @Override
    public void runOpMode() {
        initHardware();

        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("FindObjAuto7 ready");
        telemetry.update();

        waitForStart();

        led.setPosition(1);

        int cycle = 1;
        while (opModeIsActive()) {
            runPickupCycle(cycle);
            cycle++;
        }

        limelight.stop();
    }

    void runPickupCycle(int cycleNumber) {
        telemetry.addData("Cycle", cycleNumber);
        telemetry.addLine("Going to idle scan pose");
        telemetry.update();

        moveArm(IDLE_X, IDLE_Y, IDLE_Z, IDLE_PITCH_DEG, ELBOW_AUTO);
        sleep(300);
        clawOpen();

        double foundBaseDeg = scanForAprilTag();
        if (Double.isNaN(foundBaseDeg) || !opModeIsActive()) {
            return;
        }

        double centeredBaseDeg = centerOnAprilTag(foundBaseDeg);
        if (Double.isNaN(centeredBaseDeg) || !opModeIsActive()) {
            telemetry.addLine("Tag lost while centering; restarting scan");
            telemetry.update();
            moveArm(IDLE_X, IDLE_Y, IDLE_Z, IDLE_PITCH_DEG, ELBOW_AUTO);
            sleep(300);
            return;
        }

        TargetPoint target = stabilizeAndBuildTarget(centeredBaseDeg);
        if (target == null || !opModeIsActive()) {
            telemetry.addLine("Couldnt build stable target; restarting scan");
            telemetry.update();
            moveArm(IDLE_X, IDLE_Y, IDLE_Z, IDLE_PITCH_DEG, ELBOW_AUTO);
            sleep(300);
            return;
        }

        telemetry.addData("Final target", "x=%.0f y=%.0f z=%.0f", target.x, target.y, target.z);
        telemetry.addData("Range", "%.0f mm", target.rangeMm);
        telemetry.update();

        pitchServo.setPosition(SERVO_INITIAL);
        sleep(350);

        boolean movedToTag = moveArmFused(target.x, target.y, target.z, ELBOW_AUTO);
        if (!movedToTag || !opModeIsActive()) {
            telemetry.addLine("IK failed; restarting scan");
            telemetry.update();
            moveArm(IDLE_X, IDLE_Y, IDLE_Z, IDLE_PITCH_DEG, ELBOW_AUTO);
            sleep(300);
            return;
        }

        sleep(2000);
        clawClose();
        sleep(1500);

        moveArm(200, 0, 155, -45, ELBOW_AUTO);
        sleep(1000);

        telemetry.addData("Cycle complete", cycleNumber);
        telemetry.update();
    }

    double scanForAprilTag() {
        double scanPitchDeg = SCAN_PITCH_START_DEG;
        double scanAngle = -BASE_HALF;
        int scanDirection = 1;
        int sweepsAtThisPitch = 0;

        setScanPitch(scanPitchDeg);
        updateBaseOnly(scanAngle, SCAN_POWER);
        sleep(SCAN_PITCH_SETTLE_MS);

        telemetry.addLine("Scanning");
        telemetry.update();

        while (opModeIsActive()) {
            scanAngle += scanDirection * SCAN_STEP_DEG;

            if (scanAngle >= BASE_HALF) {
                scanAngle = BASE_HALF;
                scanDirection = -1;
                sweepsAtThisPitch++;
            } else if (scanAngle <= -BASE_HALF) {
                scanAngle = -BASE_HALF;
                scanDirection = 1;
                sweepsAtThisPitch++;
            }

            updateBaseOnly(scanAngle, SCAN_POWER);

            LLResultTypes.FiducialResult tag = getTargetTag();
            if (tag != null) {
                telemetry.addData("Tag found at base", "%.1f deg", scanAngle);
                telemetry.addData("Scan pitch", "%.1f deg", scanPitchDeg);
                telemetry.update();
                return scanAngle;
            }

            if (sweepsAtThisPitch >= SCAN_SWEEPS_PER_PITCH) {
                scanPitchDeg = nextScanPitch(scanPitchDeg);
                setScanPitch(scanPitchDeg);
                sweepsAtThisPitch = 0;
                sleep(SCAN_PITCH_SETTLE_MS);
            }

            telemetry.addData("Scan base", "%.1f deg", scanAngle);
            telemetry.addData("Scan pitch", "%.1f deg", scanPitchDeg);
            telemetry.addData("Sweeps at pitch", "%d/%d", sweepsAtThisPitch, SCAN_SWEEPS_PER_PITCH);
            telemetry.update();
            sleep(25);
        }

        return Double.NaN;
    }

    double nextScanPitch(double currentPitchDeg) {
        double nextPitchDeg = currentPitchDeg + SCAN_PITCH_STEP_DEG;
        if (nextPitchDeg < SCAN_PITCH_END_DEG) {
            return SCAN_PITCH_START_DEG;
        }
        return nextPitchDeg;
    }

    void setScanPitch(double pitchDeg) {
        double[] solution = solveIK(IDLE_X, IDLE_Y, IDLE_Z, pitchDeg, ELBOW_AUTO, false);
        if (solution == null) {
            return;
        }

        double pitchPos = pitchDegToServoPos(solution[3]);
        pitchServo.setPosition(clamp(pitchPos, 0.0, 1.0));
    }

    double centerOnAprilTag(double startBaseDeg) {
        double targetBaseDeg = clamp(startBaseDeg, -BASE_HALF, BASE_HALF);
        int stableFrames = 0;
        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Centering AprilTag");
        telemetry.update();

        while (opModeIsActive() && timer.milliseconds() < CENTER_TIMEOUT_MS) {
            LLResultTypes.FiducialResult tag = getTargetTag();
            if (tag == null) {
                stableFrames = 0;
                sleep(40);
                continue;
            }

            double tx = tag.getTargetXDegrees();
            if (Math.abs(tx) <= CENTER_TOLERANCE_DEG) {
                stableFrames++;
            } else {
                stableFrames = 0;
                targetBaseDeg += tx * CENTER_GAIN;
                targetBaseDeg = clamp(targetBaseDeg, -BASE_HALF, BASE_HALF);
                updateBaseOnly(targetBaseDeg, SCAN_POWER);
            }

            telemetry.addData("Center tx", "%.2f deg", tx);
            telemetry.addData("Base target", "%.1f deg", targetBaseDeg);
            telemetry.addData("Stable frames", stableFrames);
            telemetry.update();

            if (stableFrames >= 5) {
                return targetBaseDeg;
            }

            sleep(40);
        }

        return Double.NaN;
    }

    TargetPoint stabilizeAndBuildTarget(double baseDeg) {
        ElapsedTime timer = new ElapsedTime();
        double rangeSum = 0.0;
        double txSum = 0.0;
        double tySum = 0.0;
        int samples = 0;

        telemetry.addLine("Stabilizing target");
        telemetry.update();

        while (opModeIsActive() && timer.milliseconds() < STABILIZE_MS) {
            LLResult result = limelight.getLatestResult();
            LLResultTypes.FiducialResult tag = getTargetTag(result);

            if (tag != null) {
                double rangeMm = getRangeFromLimelightMm(result, tag);

                if (!Double.isNaN(rangeMm) && rangeMm > LIMELIGHT_HEIGHT_MM + 10.0) {
                    rangeSum += rangeMm;
                    txSum += tag.getTargetXDegrees();
                    tySum += tag.getTargetYDegrees();
                    samples++;
                }

                telemetry.addData("Samples", samples);
                telemetry.addData("Range now", "%.0f mm", rangeMm);
                telemetry.addData("tx", "%.2f", tag.getTargetXDegrees());
                telemetry.addData("ty", "%.2f", tag.getTargetYDegrees());
                telemetry.update();
            }

            sleep(50);
        }

        if (samples < MIN_STABLE_SAMPLES) {
            return null;
        }

        double avgRangeMm = rangeSum / samples;
        double avgTxDeg = txSum / samples;
        double avgTyDeg = tySum / samples;

        double forwardMm = Math.sqrt(Math.max(0.0,
                avgRangeMm * avgRangeMm - LIMELIGHT_HEIGHT_MM * LIMELIGHT_HEIGHT_MM));
        forwardMm += CLAW_APPROACH_OFFSET_MM;
        forwardMm -= END_EFFECTOR_BACKOFF_MM;
        forwardMm = clamp(forwardMm, MIN_TARGET_RADIUS_MM, MAX_TARGET_RADIUS_MM);

        double bearingDeg = clamp(baseDeg + avgTxDeg, -BASE_HALF, BASE_HALF);
        double bearingRad = Math.toRadians(bearingDeg);

        TargetPoint point = new TargetPoint();
        point.x = forwardMm * Math.cos(bearingRad);
        point.y = forwardMm * Math.sin(bearingRad);
        point.z = TARGET_Z_MM+17;
        point.rangeMm = avgRangeMm;
        point.bearingDeg = bearingDeg;
        point.tyDeg = avgTyDeg;
        return point;
    }

    LLResultTypes.FiducialResult getTargetTag() {
        return getTargetTag(limelight.getLatestResult());
    }

    LLResultTypes.FiducialResult getTargetTag(LLResult result) {
        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            return null;
        }

        LLResultTypes.FiducialResult best = null;
        double bestArea = -1.0;

        for (LLResultTypes.FiducialResult tag : tags) {
            if (TARGET_TAG_ID != -1 && tag.getFiducialId() != TARGET_TAG_ID) {
                continue;
            }

            if (tag.getTargetArea() > bestArea) {
                best = tag;
                bestArea = tag.getTargetArea();
            }
        }

        return best;
    }

    double getRangeFromLimelightMm(LLResult result, LLResultTypes.FiducialResult tag) {
        Pose3D targetPose = tag.getTargetPoseCameraSpace();
        double poseRangeMm = poseRangeMm(targetPose);

        if (poseRangeMm > 1.0) {
            return poseRangeMm;
        }

        double botposeAvgDistMm = result.getBotposeAvgDist() * 1000.0;
        if (botposeAvgDistMm > 1.0) {
            return botposeAvgDistMm;
        }

        double angleDownDeg = CAMERA_DOWN_ANGLE_DEG - tag.getTargetYDegrees();
        if (angleDownDeg <= 2.0 || angleDownDeg >= 88.0) {
            return Double.NaN;
        }

        return LIMELIGHT_HEIGHT_MM / Math.sin(Math.toRadians(angleDownDeg));
    }

    double poseRangeMm(Pose3D pose) {
        if (pose == null) {
            return Double.NaN;
        }

        Position position = pose.getPosition().toUnit(DistanceUnit.MM);
        return Math.sqrt(position.x * position.x + position.y * position.y + position.z * position.z);
    }

    void updateBaseOnly(double degrees, double power) {
        double clampedDeg = clamp(degrees, -BASE_HALF, BASE_HALF);
        int ticks = degreesToTicks(clampedDeg, TICKS_BASE);

        baseMotor.setTargetPosition(ticks);
        baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseMotor.setPower(power);
    }

    boolean moveArm(double x, double y, double z, double pitchDeg, int elbowPref) {
        return moveArmInternal(x, y, z, pitchDeg, elbowPref, false);
    }

    boolean moveArmFused(double x, double y, double z, int elbowPref) {
        return moveArmInternal(x, y, z, 0.0, elbowPref, true);
    }

    boolean moveArmInternal(double x, double y, double z, double pitchDeg,
                            int elbowPref, boolean holdPitchInitial) {
        double[] sol = solveIK(x, y, z, pitchDeg, elbowPref, holdPitchInitial);
        if (sol == null) {
            return false;
        }

        int baseTicks = degreesToTicks(sol[0], TICKS_BASE);
        int shoulderTicks = degreesToTicks(sol[1], TICKS_SHOULDER);
        int elbowTicks = degreesToTicks(sol[2], TICKS_ELBOW);
        double pitchPos = holdPitchInitial ? SERVO_INITIAL : pitchDegToServoPos(sol[3]);
        pitchPos = clamp(pitchPos, 0.0, 1.0);

        baseMotor.setTargetPosition(baseTicks);
        shoulderMotor.setTargetPosition(shoulderTicks);
        elbowMotor.setTargetPosition(elbowTicks);

        baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (holdPitchInitial) {
            pitchServo.setPosition(SERVO_INITIAL);
        }

        baseMotor.setPower(BASE_POWER);
        shoulderMotor.setPower(SHOULDER_POWER);
        elbowMotor.setPower(ELBOW_POWER);
        pitchServo.setPosition(pitchPos);

        telemetry.addData("Target", "x=%.0f y=%.0f z=%.0f", x, y, z);
        telemetry.addData("Base", "%.2f deg -> %d", sol[0], baseTicks);
        telemetry.addData("Shoulder", "%.2f deg -> %d", sol[1], shoulderTicks);
        telemetry.addData("Elbow", "%.2f deg -> %d", sol[2], elbowTicks);
        telemetry.addData("Pitch servo", "%.4f", pitchPos);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < MOVE_TIMEOUT_MS) {
            boolean baseDone = Math.abs(baseMotor.getCurrentPosition() - baseTicks) <= POSITION_TOLERANCE;
            boolean shoulderDone = Math.abs(shoulderMotor.getCurrentPosition() - shoulderTicks) <= POSITION_TOLERANCE;
            boolean elbowDone = Math.abs(elbowMotor.getCurrentPosition() - elbowTicks) <= POSITION_TOLERANCE;

            if (baseDone && shoulderDone && elbowDone) {
                sleep(150);
                return true;
            }

            idle();
        }

        telemetry.addLine("Move timeout");
        telemetry.update();
        return false;
    }

    double[] solveIK(double x, double y, double z, double pitchDeg,
                     int elbowPref, boolean holdPitchInitial) {
        double baseDeg = Math.toDegrees(Math.atan2(y, x));
        double r = Math.hypot(x, y);

        double cosElbow = (r * r + z * z - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
        if (cosElbow > 1.0 || cosElbow < -1.0) {
            telemetry.addData("IK fail", "Out of reach r=%.0f z=%.0f", r, z);
            telemetry.update();
            return null;
        }

        double elbowMag = Math.acos(cosElbow);
        int[] signsToTry;

        if (elbowPref == ELBOW_UP) {
            signsToTry = new int[]{1, -1};
        } else if (elbowPref == ELBOW_DOWN) {
            signsToTry = new int[]{-1, 1};
        } else {
            signsToTry = new int[]{1, -1};
        }

        double bestScore = Double.MAX_VALUE;
        double[] bestSolution = null;

        for (int sign : signsToTry) {
            double elbowRad = sign * elbowMag;
            double elbowDeg = Math.toDegrees(elbowRad);
            double shoulderRad = Math.atan2(z, r)
                    - Math.atan2(L2 * Math.sin(elbowRad),
                    L1 + L2 * Math.cos(elbowRad));
            double shoulderDeg = Math.toDegrees(shoulderRad);
            double pitchRelDeg = holdPitchInitial ? 0.0 : shoulderDeg + elbowDeg - pitchDeg;

            double score = scoreSolution(baseDeg, shoulderDeg, elbowDeg, pitchRelDeg, holdPitchInitial);
            telemetry.addData(sign > 0 ? "IK up" : "IK down",
                    score == Double.MAX_VALUE
                            ? "invalid"
                            : String.format("score=%.1f sh=%.1f el=%.1f pt=%.1f",
                            score, shoulderDeg, elbowDeg, pitchRelDeg));

            if (elbowPref != ELBOW_AUTO && score < Double.MAX_VALUE && bestSolution == null) {
                bestScore = score;
                bestSolution = new double[]{baseDeg, shoulderDeg, elbowDeg, pitchRelDeg, sign};
                continue;
            }

            if (score < bestScore) {
                bestScore = score;
                bestSolution = new double[]{baseDeg, shoulderDeg, elbowDeg, pitchRelDeg, sign};
            }
        }

        if (bestSolution == null || bestScore == Double.MAX_VALUE) {
            telemetry.addLine("IK fail: no valid solution");
            telemetry.update();
            return null;
        }

        telemetry.addData("IK chose", bestSolution[4] > 0 ? "elbow up" : "elbow down");
        telemetry.update();
        return bestSolution;
    }

    double scoreSolution(double baseDeg, double shoulderDeg,
                         double elbowDeg, double pitchRelDeg,
                         boolean holdPitchInitial) {
        if (Math.abs(baseDeg) > BASE_HALF) {
            return Double.MAX_VALUE;
        }
        if (shoulderDeg < SHOULDER_MIN || shoulderDeg > SHOULDER_MAX) {
            return Double.MAX_VALUE;
        }
        if (Math.abs(elbowDeg) > ELBOW_HALF) {
            return Double.MAX_VALUE;
        }
        if (!holdPitchInitial && Math.abs(pitchRelDeg) > PITCH_HALF) {
            return Double.MAX_VALUE;
        }

        double scoreShoulder = Math.abs(shoulderDeg - 90.0);
        double scoreElbow = Math.abs(elbowDeg);
        double scorePitch = holdPitchInitial ? 0.0 : Math.abs(pitchRelDeg);
        return scoreShoulder + scoreElbow + scorePitch;
    }

    double pitchDegToServoPos(double angleDeg) {
        return SERVO_INITIAL - angleDeg * DEG_PER_POS;
    }

    int degreesToTicks(double degrees, double ticksPerRev) {
        return (int) Math.round(degrees / 360.0 * ticksPerRev);
    }

    double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    void initHardware() {
        baseMotor = hardwareMap.get(DcMotorEx.class, BASE_MOTOR);
        shoulderMotor = hardwareMap.get(DcMotorEx.class, SHOULDER_MOTOR);
        elbowMotor = hardwareMap.get(DcMotorEx.class, ELBOW_MOTOR);
        pitchServo = hardwareMap.get(Servo.class, PITCH_SERVO);
        claw = hardwareMap.get(Servo.class, CLAW);
        claw2 = hardwareMap.get(Servo.class, CLAW2);
        rollServo = hardwareMap.get(Servo.class, ROLL_CLAW);
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT);
        led=hardwareMap.get(Servo.class, "led");

        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        baseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitchServo.setPosition(SERVO_INITIAL);
        rollServo.setPosition(ROLL_CENTER);
        clawOpen();
    }

    public void clawOpen() {
        claw.setPosition(0.0);
        claw2.setPosition(0.4);
    }

    public void clawClose() {
        claw.setPosition(0.4);
        claw2.setPosition(0.08);
    }

    static class TargetPoint {
        double x;
        double y;
        double z;
        double rangeMm;
        double bearingDeg;
        double tyDeg;
    }
}
