@file:JvmName("Commands")
package dev.frozenmilk.wavedash

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.IdentityPoseMap
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseMap
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Rotation2dDual
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.map
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait


/**
 * Builder that combines trajectories, turns, and other actions.
 */
class TrajectoryCommandBuilder private constructor(
    // constants
    private val turnCommandFactory: TurnCommandFactory,
    private val trajectoryCommandFactory: TrajectoryCommandFactory,
    private val trajectoryBuilderParams: TrajectoryBuilderParams,
    private val beginEndVel: Double,
    private val baseTurnConstraints: TurnConstraints,
    private val baseVelConstraint: VelConstraint,
    private val baseAccelConstraint: AccelConstraint,
    private val poseMap: PoseMap,
    // vary throughout
    private val tb: TrajectoryBuilder,
    private val n: Int,
    // lastPose, lastTangent are post-mapped
    private val lastPoseUnmapped: Pose2d,
    private val lastPose: Pose2d,
    private val lastTangent: Rotation2d,
    private val ms: List<MarkerFactory>,
    private val cont: (Command) -> Command,
) {
    @JvmOverloads
    constructor(
        turnActionFactory: TurnCommandFactory,
        trajectoryCommandFactory: TrajectoryCommandFactory,
        trajectoryBuilderParams: TrajectoryBuilderParams,
        beginPose: Pose2d,
        beginEndVel: Double,
        baseTurnConstraints: TurnConstraints,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        poseMap: PoseMap = IdentityPoseMap(),
    ) :
            this(
                turnActionFactory,
                trajectoryCommandFactory,
                trajectoryBuilderParams,
                beginEndVel,
                baseTurnConstraints,
                baseVelConstraint,
                baseAccelConstraint,
                poseMap,
                TrajectoryBuilder(
                    trajectoryBuilderParams,
                    beginPose, beginEndVel,
                    baseVelConstraint, baseAccelConstraint,
                    poseMap,
                ),
                0,
                beginPose,
                poseMap.map(beginPose),
                poseMap.map(beginPose).heading,
                emptyList(),
                { it },
            )

    private constructor(
        ab: TrajectoryCommandBuilder,
        tb: TrajectoryBuilder,
        n: Int,
        lastPoseUnmapped: Pose2d,
        lastPose: Pose2d,
        lastTangent: Rotation2d,
        ms: List<MarkerFactory>,
        cont: (Command) -> Command,
    ) :
            this(
                ab.turnCommandFactory,
                ab.trajectoryCommandFactory,
                ab.trajectoryBuilderParams,
                ab.beginEndVel,
                ab.baseTurnConstraints,
                ab.baseVelConstraint,
                ab.baseAccelConstraint,
                ab.poseMap,
                tb,
                n,
                lastPoseUnmapped,
                lastPose,
                lastTangent,
                ms,
                cont
            )

    /**
     * Ends the current trajectory in progress. No-op if no trajectory segments are pending.
     */
    fun endTrajectory() =
        if (n == 0) {
            require(ms.isEmpty()) { "Cannot end trajectory with pending markers" }

            this
        } else {
            val ts = tb.build()
            val endPoseUnmapped = ts.last().path.basePath.end(1).value()
            val end = ts.last().path.end(2)
            val endPose = end.value()
            val endTangent = end.velocity().value().linearVel.angleCast()
            TrajectoryCommandBuilder(
                this,
                TrajectoryBuilder(
                    trajectoryBuilderParams,
                    endPoseUnmapped,
                    beginEndVel,
                    baseVelConstraint,
                    baseAccelConstraint,
                    poseMap,
                ),
                0,
                endPoseUnmapped,
                endPose,
                endTangent,
                emptyList()
            ) { tail ->
                val (aNew, msRem) = ts.zip(ts.scan(0) { acc, t -> acc + t.offsets.size - 1 }).foldRight(
                    Pair(tail, ms)
                ) { (traj, offset), (acc, ms) ->
                    val timeTraj = TimeTrajectory(traj)
                    val actions = mutableListOf(seqCons(trajectoryCommandFactory.make(timeTraj), acc))
                    val msRem = mutableListOf<MarkerFactory>()
                    for (m in ms) {
                        val i = m.segmentIndex - offset
                        if (i >= 0) {
                            actions.add(m.make(timeTraj, traj.offsets[i]))
                        } else {
                            msRem.add(m)
                        }
                    }

                    if (actions.size == 1) {
                        Pair(actions.first(), msRem)
                    } else {
                        Pair(Parallel(actions), msRem)
                    }
                }

                require(msRem.isEmpty()) { "Unresolved markers" }

                cont(aNew)
            }
        }

    /**
     * Stops the current trajectory (like [endTrajectory]) and adds command [command] next.
     */
    fun stopAndAdd(command: Command): TrajectoryCommandBuilder {
        val b = endTrajectory()
        return TrajectoryCommandBuilder(b, b.tb, b.n, b.lastPoseUnmapped, b.lastPose, b.lastTangent, b.ms) { tail ->
            b.cont(seqCons(command, tail))
        }
    }

    /**
     * Waits [t] seconds.
     */
    fun waitSeconds(t: Double): TrajectoryCommandBuilder {
        require(t >= 0.0) { "Time ($t) must be non-negative" }

        return stopAndAdd(Wait(t))
    }

    /**
     * Schedules action [command] to execute in parallel starting at a displacement [ds] after the last trajectory segment.
     * The action start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    // TODO: Should calling this without an applicable trajectory implicitly begin an empty trajectory and execute the
    // action immediately?
    fun afterDisp(ds: Double, command: Command): TrajectoryCommandBuilder {
        require(ds >= 0.0) { "Displacement ($ds) must be non-negative" }

        return TrajectoryCommandBuilder(
            this, tb, n, lastPoseUnmapped, lastPose, lastTangent,
            ms + listOf(DispMarkerFactory(n, ds, command)), cont
        )
    }

    /**
     * Schedules action [command] to execute in parallel starting [dt] seconds after the last trajectory segment, turn, or
     * other action.
     */
    fun afterTime(dt: Double, command: Command): TrajectoryCommandBuilder {
        require(dt >= 0.0) { "Time ($dt) must be non-negative" }

        return if (n == 0) {
            TrajectoryCommandBuilder(this, tb, 0, lastPoseUnmapped, lastPose, lastTangent, emptyList()) { tail ->
                cont(Parallel(tail, seqCons(Wait(dt), command)))
            }
        } else {
            TrajectoryCommandBuilder(
                this, tb, n, lastPoseUnmapped, lastPose, lastTangent,
                ms + listOf(TimeMarkerFactory(n, dt, command)), cont
            )
        }
    }

    /**
     * Schedules [command] to execute in parallel starting at a time [dt] before the end of the last trajectory
     * segment.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    fun untilTime(dt: Double, command: Command): TrajectoryCommandBuilder {
        require(dt >= 0.0) { "Time ($dt) must be non-negative" }

        return TrajectoryCommandBuilder(
            this, tb, n, lastPoseUnmapped, lastPose, lastTangent,
            ms + listOf(UntilTimeMarkerFactory(n, dt, command)), cont
        )
    }

    /**
     * Schedules [command] to execute in parallel starting at a displacement [ds] before the end of the last trajectory
     * segment.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    fun untilDisp(ds: Double, command: Command): TrajectoryCommandBuilder {
        require(ds >= 0.0) { "Displacement ($ds) must be non-negative" }

        return TrajectoryCommandBuilder(
            this, tb, n, lastPoseUnmapped, lastPose, lastTangent,
            ms + listOf(UntilDispMarkerFactory(n, ds, command)), cont
        )
    }

    /**
     * Schedules [command] to execute in parallel starting with the last trajectory segment.
     * This is equivalent to [afterTime] with a dt of 0.
     */
    fun duringLast(command: Command) = afterTime(0.0, command)

    fun setTangent(r: Rotation2d) =
        TrajectoryCommandBuilder(this, tb.setTangent(r), n, lastPoseUnmapped, lastPose, lastTangent, ms, cont)
    fun setTangent(r: Double) = setTangent(Rotation2d.exp(r))

    fun setReversed(reversed: Boolean) =
        TrajectoryCommandBuilder(this, tb.setReversed(reversed), n, lastPoseUnmapped, lastPose, lastTangent, ms, cont)

    @JvmOverloads
    fun turn(angle: Double, turnConstraintsOverride: TurnConstraints? = null): TrajectoryCommandBuilder {
        val b = endTrajectory()
        val mappedAngle =
            poseMap.map(
                Pose2dDual(
                    Vector2dDual.constant(b.lastPose.position, 2),
                    Rotation2dDual.constant<Arclength>(b.lastPose.heading, 2) + DualNum(listOf(0.0, angle))
                )
            ).heading.velocity().value()
        val b2 = b.stopAndAdd(
            turnCommandFactory.make(
                TimeTurn(b.lastPose, mappedAngle, turnConstraintsOverride ?: baseTurnConstraints)
            )
        )
        val lastPoseUnmapped = Pose2d(b2.lastPoseUnmapped.position, b2.lastPoseUnmapped.heading + angle)
        val lastPose = Pose2d(b2.lastPose.position, b2.lastPose.heading + mappedAngle)
        val lastTangent = b2.lastTangent + mappedAngle
        return TrajectoryCommandBuilder(
            b2,
            TrajectoryBuilder(
                trajectoryBuilderParams,
                lastPoseUnmapped,
                beginEndVel,
                baseVelConstraint,
                baseAccelConstraint,
                poseMap
            ),
            b2.n, lastPoseUnmapped, lastPose, lastTangent, b2.ms, b2.cont
        )
    }
    @JvmOverloads
    fun turnTo(heading: Rotation2d, turnConstraintsOverride: TurnConstraints? = null): TrajectoryCommandBuilder {
        val b = endTrajectory()
        return b.turn(heading - b.lastPose.heading, turnConstraintsOverride)
    }
    @JvmOverloads
    fun turnTo(heading: Double, turnConstraintsOverride: TurnConstraints? = null) =
        turnTo(Rotation2d.exp(heading), turnConstraintsOverride)

    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToX(
            posX, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXConstantHeading(
            posX, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXLinearHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXLinearHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXSplineHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXSplineHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToY(
            posY, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYConstantHeading(
            posY, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYLinearHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYLinearHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYSplineHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYSplineHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun strafeTo(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeTo(
            pos, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun strafeToConstantHeading(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToConstantHeading(
            pos, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToLinearHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToLinearHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToSplineHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToSplineHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineTo(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineTo(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToConstantHeading(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToConstantHeading(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToLinearHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToLinearHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToSplineHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToSplineHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Creates a new builder with the same settings at the current pose, tangent.
     */
    fun fresh() = endTrajectory().let {
        TrajectoryCommandBuilder(
            it.turnCommandFactory,
            it.trajectoryCommandFactory,
            it.trajectoryBuilderParams,
            it.lastPoseUnmapped,
            it.beginEndVel, it.baseTurnConstraints, it.baseVelConstraint, it.baseAccelConstraint,
            it.poseMap
        ).setTangent(it.lastTangent)
    }

    fun build(): Command {
        return endTrajectory().cont(Sequential())
    }
}

@JvmField
val DEFAULT_TRAJECTORY_PARAMS = TrajectoryBuilderParams(
    1e-6,
    ProfileParams(
        0.25, 0.1, 1e-2
    )
)

private fun seqCons(hd: Command, tl: Command): Command =
    if (tl is Sequential) {
        Sequential(listOf(hd) + tl.commands)
    } else {
        Sequential(hd, tl)
    }

private abstract class MarkerFactory(
    val segmentIndex: Int,
) {
    abstract fun make(t: TimeTrajectory, segmentDisp: Double): Command
}

private class TimeMarkerFactory(segmentIndex: Int, val dt: Double, val command: Command) : MarkerFactory(segmentIndex) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        seqCons(Wait(t.profile.inverse(segmentDisp) + dt), command)
}

private class DispMarkerFactory(segmentIndex: Int, val ds: Double, val command: Command) : MarkerFactory(segmentIndex) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        seqCons(Wait(t.profile.inverse(segmentDisp + ds)), command)
}

private class UntilTimeMarkerFactory(segmentIndex: Int, val dt: Double, val command: Command) : MarkerFactory(segmentIndex) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        seqCons(Wait(t.profile.inverse(segmentDisp) + t.duration - dt), command)
}

private class UntilDispMarkerFactory(segmentIndex: Int, val ds: Double, val command: Command) : MarkerFactory(segmentIndex) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        seqCons(Wait(t.profile.inverse(segmentDisp + t.profile.dispProfile.length - ds)), command)
}

fun interface TurnCommandFactory {
    fun make(t: TimeTurn): Command
}

fun interface TrajectoryCommandFactory {
    fun make(t: TimeTrajectory): Command
}