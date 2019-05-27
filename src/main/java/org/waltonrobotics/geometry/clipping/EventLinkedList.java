package org.waltonrobotics.geometry.clipping;

public class EventLinkedList {

    public EventNode root;

    public interface FindTransitionCheck {

        /* Search the list.
           Keep checking for a condition
           Keep returning false until the condition is true
         */
        boolean check(EventNode node);

    }

    public interface InsertBeforeCheck {

        /* Perform some check between `data` and the current `node.`
           if we want to insert `data` before `node`, return true
           otherwise, return false.
         */
        boolean check(EventNode data, EventNode node);

    }

    public class Transition {
        public EventNode prev;
        public EventNode here;
        public EventNode before;
        public EventNode after;

        public EventNode insert(EventNode node) {
            node.prev = prev;
            node.next = here;
            prev.next = node;
            if (here != null) {
                here.prev = node;
            }
            return node;
        }
    }

    public EventLinkedList() {
        root = new EventNode();
        root.root = true;
        root.next = null;
    }

    public boolean exists(EventNode node) {
        return node != null && node != root;
    }

    public boolean isEmpty() {
        return root.next == null;
    }

    public EventNode getHead() {
        return root.next;
    }

    public void insertBefore(EventNode node, InsertBeforeCheck check) {
        EventNode last = root;
        EventNode here = root.next;

        while (here != null) {
            if (check.check(node, here)) {
                node.prev = here.prev;
                node.next = here;
                here.prev.next = node;
                here.prev = node;
                return;
            }
            last = here;
            here = here.next;
        }

        last.next = node;
        node.prev = last;
        node.next = null;
    }

    public Transition findTransition(FindTransitionCheck check) {
        EventNode prev = root;
        EventNode here = root.next;

        while (here != null) {
            if (check.check(here)) {
                break;
            }

            prev = here;
            here = here.next;
        }

        Transition ret = new Transition();
        ret.prev = prev;
        ret.here = here;
        ret.before = (prev == root ? null : prev);
        ret.after = here;
        return ret;
    }

}
