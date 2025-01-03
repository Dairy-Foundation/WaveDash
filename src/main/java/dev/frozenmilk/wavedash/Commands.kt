package dev.frozenmilk.wavedash

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.IdentityPoseMap
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseMap
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Rotation2dDual
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.VelConstraint
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait

/**
 * Builder that combines trajectories, turns, and other Commands.
 */
class TrajectoryCommandBuilder
@JvmOverloads constructor(
    private val trajectoryFactory: (Trajectory) -> Command,
    private val turnFactory: (TimeTurn) -> Command,
    beginPose: Pose2d,
    private val params: TrajectoryBuilderParams,
    private val baseVelConstraint: VelConstraint,
    private val baseAccelConstraint: AccelConstraint,
    private val baseTurnConstraint: TurnConstraints,
    private val poseMap: PoseMap = IdentityPoseMap(),
) {
    private var builder = TrajectoryBuilder(
        params,
        beginPose,
        0.0,
        baseVelConstraint,
        baseAccelConstraint
    )

    var size = 0
        private set

    private val commands: MutableList<Command> = mutableListOf()

    fun endTrajectory(): TrajectoryCommandBuilder {
        if (size != commands.size) {
            val built = builder.build()
            built
                .map { trajectoryFactory(it) }
                .forEach { commands.add(it) }

            val lastPose = built.last().path.end(1).value()

            builder = TrajectoryBuilder(
                params,
                lastPose,
                0.0,
                baseVelConstraint,
                baseAccelConstraint,
                poseMap
            )
        }

        return this
    }

    fun fresh(): TrajectoryCommandBuilder {
        val built = builder.build()

        val lastPose = built.last().path.end(1).value()

        return TrajectoryCommandBuilder(
            trajectoryFactory,
            turnFactory,
            lastPose,
            params,
            baseVelConstraint,
            baseAccelConstraint,
            baseTurnConstraint,
            poseMap,
        )
    }

    fun build(): Command {
        if (size != commands.size) {
            endTrajectory()
        }

        return Sequential(commands)
    }

    fun afterTime(time: Double, command: Command): TrajectoryCommandBuilder {
        endTrajectory()

        val last = commands.removeAt(commands.lastIndex)
        commands += Parallel(last, Sequential(Wait(time), command))

        return this
    }

    fun afterDisp(distance: Double, command: Command): TrajectoryCommandBuilder {
        val built = builder.build()

        val time = TimeProfile(built.last().profile.baseProfile).inverse(distance)

        return this.afterTime(time, command)
    }

    fun stopAndAdd(command: Command): TrajectoryCommandBuilder {
        size += 1
        endTrajectory()
        commands += command
        return this
    }

    fun setTangent(tangent: Rotation2d): TrajectoryCommandBuilder {
        builder = builder.setTangent(tangent)
        return this
    }

    fun setTangent(tangent: Double): TrajectoryCommandBuilder {
        return this.setTangent(Rotation2d.exp(tangent))
    }

    fun setReversed(reversed: Boolean): TrajectoryCommandBuilder {
        builder = builder.setReversed(reversed)
        return this
    }

    @JvmOverloads
    fun splineTo(
        point: Vector2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1
        builder = builder.splineTo(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun splineTo(
        point: Vector2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.splineTo(point, Rotation2d.exp(tangent), velOverride, accelOverride)
    }

    @JvmOverloads
    fun splineToSplineHeading(
        point: Pose2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1
        builder = builder.splineToSplineHeading(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun splineToSplineHeading(
        point: Pose2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.splineToSplineHeading(
            point,
            Rotation2d.exp(tangent),
            velOverride,
            accelOverride
        )
    }

    @JvmOverloads
    fun splineToLinearHeading(
        point: Pose2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1
        builder = builder.splineToLinearHeading(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun splineToLinearHeading(
        point: Pose2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.splineToLinearHeading(
            point,
            Rotation2d.exp(tangent),
            velOverride,
            accelOverride
        )
    }

    @JvmOverloads
    fun splineToConstantHeading(
        point: Vector2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1
        builder = builder.splineToConstantHeading(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun splineToConstantHeading(
        point: Vector2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.splineToConstantHeading(
            point,
            Rotation2d.exp(tangent),
            velOverride,
            accelOverride
        )
    }

    @JvmOverloads
    fun strafeTo(
        point: Vector2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1
        builder = builder.strafeTo(point, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun strafeToSplineHeading(
        point: Vector2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1
        builder = builder.strafeToSplineHeading(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun strafeToSplineHeading(
        point: Vector2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.strafeToSplineHeading(
            point,
            Rotation2d.exp(tangent),
            velOverride,
            accelOverride
        )
    }

    @JvmOverloads
    fun strafeToLinearHeading(
        point: Vector2d,
        tangent: Rotation2d,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        size += 1
        builder = builder.strafeToLinearHeading(point, tangent, velOverride, accelOverride)
        return this
    }

    @JvmOverloads
    fun strafeToLinearHeading(
        point: Vector2d,
        tangent: Double,
        velOverride: VelConstraint? = null,
        accelOverride: AccelConstraint? = null
    ): TrajectoryCommandBuilder {
        return this.strafeToLinearHeading(
            point,
            Rotation2d.exp(tangent),
            velOverride,
            accelOverride
        )
    }

    @JvmOverloads
    fun turn(
        angle: Double,
        turnOverride: TurnConstraints? = null
    ): TrajectoryCommandBuilder {
        size += 1

        val built = builder.build()

        val lastPose = built.last().path.end(1).value()

        val mappedAngle = poseMap.map(
            Pose2dDual(
                Vector2dDual.constant(lastPose.position, 2),
                Rotation2dDual.constant<Arclength>(lastPose.heading, 2) + DualNum(listOf(0.0, angle))
            )
        ).heading.velocity().value()

        return this.stopAndAdd(
            turnFactory(
                TimeTurn(
                    lastPose,
                    mappedAngle,
                    turnOverride ?: baseTurnConstraint
                )
            )
        )
    }

    @JvmOverloads
    fun turn(
        angle: Rotation2d,
        turnOverride: TurnConstraints? = null
    ): TrajectoryCommandBuilder {
        return this.turn(angle.toDouble(), turnOverride)
    }

    @JvmOverloads
    fun turnTo(
        angle: Double,
        turnOverride: TurnConstraints? = null
    ): TrajectoryCommandBuilder {
        size += 1

        val built = builder.build()

        val lastPose = built.last().path.end(1).value()

        return this.turn(angle - lastPose.heading.toDouble(), turnOverride)
    }

    @JvmOverloads
    fun turnTo(
        angle: Rotation2d,
        turnOverride: TurnConstraints? = null
    ): TrajectoryCommandBuilder {
        return this.turnTo(angle.toDouble(), turnOverride)
    }
}