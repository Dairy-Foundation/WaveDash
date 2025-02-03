package dev.frozenmilk.wavedash

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d

/**
 * Interface for localization methods.
 */
interface Localizer {
    /**
     * The current pose according to the Localizer.
     * NOTE: This pose is only changed when the `update`
     * method is called.
     * @return the Localizer's current pose
     */
    var pose: Pose2d

    /**
     * Updates the Localizer's pose estimate.
     * @return the Localizer's current velocity estimate
     */
    fun update(): PoseVelocity2d
}
