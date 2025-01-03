package dev.frozenmilk.wavedash;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.util.OpModeLazyCell;
import dev.frozenmilk.wavedash.messages.MecanumLocalizerInputsMessage;

public class MecanumDriveLocalizer implements Localizer {
    private final MecanumParams params;
    private final MecanumKinematics kinematics;
    public final Encoder leftFront, leftBack, rightBack, rightFront;
    public final IMU imu;

    private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
    private Rotation2d lastHeading;
    private boolean initialized;
    private Pose2d pose;

    public MecanumDriveLocalizer(MecanumParams params) {
        HardwareMap hwMap = FeatureRegistrar.getActiveOpMode().hardwareMap;

        this.params = params;
        leftFront = new OpModeLazyCell<>(
                () -> new OverflowEncoder(new RawEncoder(hwMap.get(DcMotorEx.class, params.lfName)))).get();
        leftBack = new OpModeLazyCell<>(
                () -> new OverflowEncoder(new RawEncoder(hwMap.get(DcMotorEx.class, params.lbName)))).get();
        rightBack = new OpModeLazyCell<>(
                () -> new OverflowEncoder(new RawEncoder(hwMap.get(DcMotorEx.class, params.rbName)))).get();
        rightFront = new OpModeLazyCell<>(
                () -> new OverflowEncoder(new RawEncoder(hwMap.get(DcMotorEx.class, params.rfName)))).get();

        imu = new OpModeLazyCell<>(
                () -> new LazyImu(hwMap, params.imuName, new RevHubOrientationOnRobot(params.logoFacingDirection, params.usbFacingDirection))).get().get();

        kinematics = new MecanumKinematics(
                params.inPerTick * params.trackWidthTicks, params.inPerTick / params.lateralInPerTick);

        // TODO: reverse encoders if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
        PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
        PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
        PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        if (!initialized) {
            initialized = true;

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        double headingDelta = heading.minus(lastHeading);
        Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                new DualNum<Time>(new double[]{
                        (leftFrontPosVel.position - lastLeftFrontPos),
                        leftFrontPosVel.velocity,
                }).times(params.inPerTick),
                new DualNum<Time>(new double[]{
                        (leftBackPosVel.position - lastLeftBackPos),
                        leftBackPosVel.velocity,
                }).times(params.inPerTick),
                new DualNum<Time>(new double[]{
                        (rightBackPosVel.position - lastRightBackPos),
                        rightBackPosVel.velocity,
                }).times(params.inPerTick),
                new DualNum<Time>(new double[]{
                        (rightFrontPosVel.position - lastRightFrontPos),
                        rightFrontPosVel.velocity,
                }).times(params.inPerTick)
        ));

        lastLeftFrontPos = leftFrontPosVel.position;
        lastLeftBackPos = leftBackPosVel.position;
        lastRightBackPos = rightBackPosVel.position;
        lastRightFrontPos = rightFrontPosVel.position;

        lastHeading = heading;

        pose = pose.plus(new Twist2d(
                twist.line.value(),
                headingDelta
        ));

        return twist.velocity().value();
    }
}
