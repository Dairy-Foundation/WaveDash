package dev.frozenmilk.wavedash

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.Vector2d
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda

interface Drive {
    val localizer: Localizer

    fun setDrivePowers(powers: PoseVelocity2d)
    fun setDrivePowersWithFF(powers: PoseVelocity2d)

    fun followTrajectory(trajectory: TimeTrajectory, t: Double): Boolean
    fun turn(turn: TimeTurn, t: Double): Boolean

    fun commandBuilder(beginPose: Pose2d): TrajectoryCommandBuilder

    fun followTrajectoryCommand(trajectory: TimeTrajectory): Command {
        val requirements = setOf(this)
        val states: Set<Wrapper.OpModeState> = setOf(Wrapper.OpModeState.ACTIVE)
        val start = System.nanoTime() * 1e-9
        var t = 0.0
        var finished = false

        return Lambda("Follow Trajectory Command")
            .setRequirements(requirements)
            .setRunStates(states)
            .setExecute {
                finished = this.followTrajectory(trajectory, t)
                t = start - System.nanoTime()
            }.setFinish { finished }
            .setEnd {
                this.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            }
    }

    fun turnCommand(turn: TimeTurn): Command {
        val requirements = setOf(this)
        val states: Set<Wrapper.OpModeState> = setOf(Wrapper.OpModeState.ACTIVE)
        val start = System.nanoTime() * 1e-9
        var t = 0.0
        var finished = false

        return Lambda("Turn Command")
            .setRequirements(requirements)
            .setRunStates(states)
            .setExecute {
                finished = this.turn(turn, t)
                t = start - System.nanoTime()
            }.setFinish { finished }
            .setEnd {
                this.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            }
    }
}

