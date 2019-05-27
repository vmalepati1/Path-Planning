package org.waltonrobotics.geometry.clipping;

import org.waltonrobotics.geometry.Vector2f;

public class Event {

    public boolean isStart;
    public Vector2f pt;
    public Segment seg;
    public Event other;
    public EventNode status;

}
