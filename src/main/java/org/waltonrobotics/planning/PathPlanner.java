package org.waltonrobotics.planning;

import org.waltonrobotics.geometry.Pose;

import java.util.List;

public interface PathPlanner {

    List<Pose> findPath(Pose startingPose, Pose endingPose);

}
