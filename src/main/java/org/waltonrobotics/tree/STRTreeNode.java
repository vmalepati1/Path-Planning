package org.waltonrobotics.tree;

import org.waltonrobotics.geometry.Rectangle;

import java.util.LinkedList;

public class STRTreeNode {

    public STRTreeNode parent;
    public LinkedList<STRTreeNode> children;
    public STRTree hostTree;
    public Rectangle boundingBox;

    public STRTreeNode(STRTreeNode parent, LinkedList<STRTreeNode> children, STRTree hostTree, Rectangle boundingBox) {
        this.parent = parent;
        this.children = children;
        this.hostTree = hostTree;
        this.boundingBox = boundingBox;
    }

}
