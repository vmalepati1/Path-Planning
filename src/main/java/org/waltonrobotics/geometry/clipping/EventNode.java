package org.waltonrobotics.geometry.clipping;

public class EventNode {

    public boolean root;
    public EventNode prev;
    public EventNode next;
    public Event event;

    public EventNode() {
        root = false;
        prev = null;
        next = null;
        event = null;
    }

    public void remove() {
        prev.next = next;
        if (next != null) {
            next.prev = prev;
        }
        prev = null;
        next = null;
    }

}
