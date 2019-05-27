package org.waltonrobotics.geometry.clipping;

import org.waltonrobotics.geometry.Vector2f;

import java.util.List;

public class PolygonClipper {

    private EventLinkedList eventRoot = new EventLinkedList();
    private Epsilon eps;

    public PolygonClipper(Epsilon eps) {
        this.eps = eps;
    }

    public void eventAddRegion(List<Vector2f> region) {
        Vector2f pt1;
        Vector2f pt2 = region.get(region.size() - 1);

        for (int i = 0; i < region.size(); i++) {
            pt1 = pt2;
            pt2 = region.get(i);

            int forward = eps.pointsCompare(pt1, pt2);

            if (forward == 0) {
                continue;
            }

            eventAddSegment(new Segment(forward < 0 ? pt1 : pt2, forward < 0 ? pt2 : pt1));
        }
    }

    private Event eventAddSegment(Segment seg) {
        Event evStart = eventAddSegmentStart(seg);
        eventAddSegmentEnd(evStart, seg);
        return evStart;
    }

    private Event eventAddSegmentStart(Segment seg) {
        Event evStart = new Event();
        evStart.isStart = true;
        evStart.pt = seg.getStart();
        evStart.seg = seg;
        evStart.other = null;
        evStart.status = null;

        eventAdd(evStart, seg.getEnd());
        return evStart;
    }

    private void eventAddSegmentEnd(Event evStart, Segment seg) {
        Event evEnd = new Event();
        evEnd.isStart = false;
        evEnd.pt = seg.getEnd();
        evEnd.seg = seg;
        evEnd.other = evStart;
        evEnd.status = null;

        evStart.other = evEnd;
        eventAdd(evEnd, evStart.pt);
    }

    private void eventAdd(Event ev, Vector2f otherPt) {
        EventNode eventNode = new EventNode();
        eventNode.event = ev;

        eventRoot.insertBefore(eventNode, new EventAddCheck(otherPt));
    }

    private class EventAddCheck implements EventLinkedList.InsertBeforeCheck {

        Vector2f otherPt;

        public EventAddCheck(Vector2f otherPt) {
            this.otherPt = otherPt;
        }

        @Override
        public boolean check(EventNode data, EventNode node) {
            int comp = eventCompare(data.event.isStart, data.event.pt, otherPt,
                    node.event.isStart, node.event.pt, node.event.other.pt);

            return comp < 0;
        }

        private int eventCompare(boolean p1IsStart, Vector2f p1, Vector2f p2, boolean p2IsStart, Vector2f p3, Vector2f p4) {
            int comp = eps.pointsCompare(p1, p3);
            if (comp != 0) {
                return comp;
            }

            if (eps.pointsCompare(p2, p4) == 0) {
                return 0;
            }

            if (p1IsStart != p2IsStart) {
                return p1IsStart ? 1 : -1;
            }

            return eps.pointAboveOrOnLine(p2, p3, p4) ? 1 : -1;
        }

    }

}
