package dev.frozenmilk.wavedash;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import dev.frozenmilk.wavedash.messages.DriveCommandMessage;
import dev.frozenmilk.wavedash.messages.MecanumCommandMessage;
import dev.frozenmilk.wavedash.messages.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;

@Config
public class DefaultMecanumDrive implements Drive {

    public MecanumParams params;

    public final MecanumKinematics kinematics;

    public final TurnConstraints defaultTurnConstraints;
    public final VelConstraint defaultVelConstraint;

    public final AccelConstraint defaultAccelConstraint;

    public final DcMotorEx leftFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightBack;

    public final DcMotorEx rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public DefaultMecanumDrive(HardwareMap hardwareMap, MecanumParams params) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.params = params;

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, this.params.lfName);
        leftBack = hardwareMap.get(DcMotorEx.class, this.params.lbName);
        rightBack = hardwareMap.get(DcMotorEx.class, this.params.rbName);
        rightFront = hardwareMap.get(DcMotorEx.class, this.params.rfName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(params.lfDirection);
        leftBack.setDirection(params.lbDirection);
        rightBack.setDirection(params.rbDirection);
        rightFront.setDirection(params.rfDirection);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyImu(hardwareMap, this.params.imuName, new RevHubOrientationOnRobot(
                this.params.logoFacingDirection, this.params.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = this.params.localizer;
        localizer.setPose(this.params.beginPose);

        FlightRecorder.write("MECANUM_PARAMS", this.params);

        kinematics = new MecanumKinematics(
                this.params.inPerTick * this.params.trackWidthTicks, this.params.inPerTick / this.params.lateralInPerTick);

        defaultVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(this.params.maxWheelVel),
                        new AngularVelConstraint(this.params.maxAngVel)
                ));

        defaultTurnConstraints = new TurnConstraints(
                this.params.maxAngVel, -this.params.maxAngAccel, this.params.maxAngAccel);

        defaultAccelConstraint =
                new ProfileAccelConstraint(this.params.minProfileAccel, this.params.maxProfileAccel);
    }

    @NonNull
    @Override
    public Localizer getLocalizer() {
        return localizer;
    }

    @Override
    public void setDrivePowers(@NonNull PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    @Override
    public void setDrivePowersWithFF(@NonNull PoseVelocity2d powers) {
        setDrivePowersWithFF(PoseVelocity2dDual.constant(powers, 1));
    }

    public void setDrivePowersWithFF(PoseVelocity2dDual<Time> powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(powers);
        double voltage = voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(params.kS,
                params.kV / params.inPerTick, params.kA / params.inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());
        
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        
        
        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d beginPose) {
        return new TrajectoryBuilder(
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose,
                0.0,
                defaultVelConstraint,
                defaultAccelConstraint
        );
    }

    /**
     * Follow a trajectory.
     * @param trajectory trajectory to follow
     * @param t time to follow in seconds
     * @return whether the trajectory has been completed
     */
    public boolean followTrajectory(TimeTrajectory trajectory, double t) {
        if (t >= trajectory.duration) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            return true;
        }

        Pose2dDual<Time> txWorldTarget = trajectory.get(t);
        targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

        PoseVelocity2d robotVelRobot = updatePoseEstimate();

        PoseVelocity2dDual<Time> command = new HolonomicController(
                params.axialGain, params.lateralGain, params.headingGain,
                params.axialVelGain, params.lateralVelGain, params.headingVelGain
        )
                .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
        driveCommandWriter.write(new DriveCommandMessage(command));

        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(params.kS,
                params.kV / params.inPerTick, params.kA / params.inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        return false;
    }

    /**
     * Follow a trajectory.
     * @param trajectory trajectory to follow
     * @param t time to follow in seconds
     * @return whether the trajectory has been completed
     **/
    public boolean followTrajectory(@NonNull Trajectory trajectory, double t) {
        return followTrajectory(new TimeTrajectory(trajectory), t);
    }

    @Override
    public boolean turn(TimeTurn turn, double t) {
        if (t >= turn.duration) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            return true;
        }

        Pose2dDual<Time> txWorldTarget = turn.get(t);
        targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

        PoseVelocity2d robotVelRobot = updatePoseEstimate();

        PoseVelocity2dDual<Time> command = new HolonomicController(
                params.axialGain, params.lateralGain, params.headingGain,
                params.axialVelGain, params.lateralVelGain, params.headingVelGain
        )
                .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
        driveCommandWriter.write(new DriveCommandMessage(command));

        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();
        final MotorFeedforward feedforward = new MotorFeedforward(params.kS,
                params.kV / params.inPerTick, params.kA / params.inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
        leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
        rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
        rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

        return false;
    }

    @NonNull
    @Override
    public TrajectoryCommandBuilder commandBuilder(@NonNull Pose2d beginPose) {
        return new TrajectoryCommandBuilder(
                this::followTrajectoryCommand,
                this::turnCommand,
                beginPose,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                defaultVelConstraint,
                defaultAccelConstraint,
                defaultTurnConstraints
        );
    }
}
