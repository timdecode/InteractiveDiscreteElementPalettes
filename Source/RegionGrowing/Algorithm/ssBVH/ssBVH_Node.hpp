﻿// Copyright(C) David W. Jeske, 2014, and released to the public domain.
//
// Dynamic BVH (Bounding Volume Hierarchy) using incremental refit and tree-rotations
//
// initial BVH build based on: Bounding Volume Hierarchies (BVH) – A brief tutorial on what they are and how to implement them
//              http://www.3dmuve.com/3dmblog/?p=182
//
// Dynamic Updates based on: "Fast, Effective BVH Updates for Animated Scenes" (Kopta, Ize, Spjut, Brunvand, David, Kensler)
//              http://www.cs.utah.edu/~thiago/papers/rotations.pdf
//
// see also:  Space Partitioning: Octree vs. BVH
//            http://thomasdiewald.com/blog/?p=1488
//
//

// TODO: implement node merge/split, to handle updates when LEAF_OBJ_MAX > 1
//

#include <vector>

#include "ssBVH.hpp"

namespace dynamicBVH
{
    template <typename ObjectT>
    class BVHNode
    {
    public:
        AABB box;
        
        BVHNode<ObjectT> * parent = nullptr;
        BVHNode<ObjectT> * left = nullptr;
        BVHNode<ObjectT> * right = nullptr;
        
        int depth;
        
        std::vector<ObjectT> gobjects;  // only populated in leaf nodes
       
        
        BVHNode(const std::vector<ObjectT>& gobjects_in) : gobjects(gobjects_in) {}
        
        BVHNode(BVHNode<ObjectT> * parent_in,
                std::vector<ObjectT> gobjects_in,
                Axis lastSplitAxis,
                int depth_in)
        {
            parent = parent_in;
            depth = depth_in;
            
            // Check if we’re at our LEAF node, and if so, save the objects and stop recursing.  Also store the min/max for the leaf node and update the parent appropriately
            if( gobjectlist.size() <= bvh.LEAF_OBJ_MAX )
            {
                // once we reach the leaf node, we must set prev/next to null to signify the end
                left = nullptr;
                right = nullptr;
                
                // at the leaf node we store the remaining objects, so initialize a list
                gobjects = gobjects_in;
                
                computeVolume(nAda);
                splitIfNecessary(nAda);
            }
            else
            {
                // --------------------------------------------------------------------------------------------
                // if we have more than (bvh.LEAF_OBJECT_COUNT) objects, then compute the volume and split
                gobjects = gobjectlist;
                computeVolume(nAda);
                splitNode(nAda);
                childRefit(nAda,recurse:false);
            }
        }
        
        Axis pickSplitAxis()
        {
            float axis_x = box.Max.X - box.Min.X;
            float axis_y = box.Max.Y - box.Min.Y;
            float axis_z = box.Max.Z - box.Min.Z;
            
            // return the biggest axis
            if (axis_x > axis_y)
            {
                if (axis_x > axis_z)
                    return Axis.X;
                else
                    return Axis.Z;
            }
            else
            {
                if (axis_y > axis_z)
                    return Axis.Y;
                else
                    return Axis.Z;
            }
        }
        
        bool isLeaf()
        {
            return gobjects.size() > 0;
        }
        
        Axis nextAxis(Axis cur)
        {
            return cur % Axis.End;
        }
        
        
        void expandVolume(SSBVHNodeAdaptor<ObjectT> nAda, Vector3 objectpos, float radius) {
            bool expanded = false;
            
            // test min X and max X against the current bounding volume
            if ((objectpos.X - radius) < box.Min.X) {
                box.Min.X = (objectpos.X - radius);
                expanded = true;
            }
            if ((objectpos.X + radius) > box.Max.X) {
                box.Max.X = (objectpos.X + radius);
                expanded = true;
            }
            // test min Y and max Y against the current bounding volume
            if ((objectpos.Y - radius) < box.Min.Y) {
                box.Min.Y = (objectpos.Y - radius);
                expanded = true;
            }
            if ((objectpos.Y + radius) > box.Max.Y) {
                box.Max.Y = (objectpos.Y + radius);
                expanded = true;
            }
            // test min Z and max Z against the current bounding volume
            if ( (objectpos.Z - radius) < box.Min.Z ) {
                box.Min.Z = (objectpos.Z - radius);
                expanded = true;
            }
            if ( (objectpos.Z + radius) > box.Max.Z ) {
                box.Max.Z = (objectpos.Z + radius);
                expanded = true;
            }
            
            if (expanded && parent != null) {
                parent.childExpanded(nAda, this);
            }
        }
        
        private void assignVolume(Vector3 objectpos, float radius)
        {
            box.Min.X = objectpos.X - radius;
            box.Max.X = objectpos.X + radius;
            box.Min.Y = objectpos.Y - radius;
            box.Max.Y = objectpos.Y + radius;
            box.Min.Z = objectpos.Z - radius;
            box.Max.Z = objectpos.Z + radius;
        }
        
        void computeVolume(SSBVHNodeAdaptor<ObjectT> nAda)
        {
            assignVolume( nAda.objectpos(gobjects[0]), nAda.radius(gobjects[0]));
            for(int i=1; i<gobjects.Count;i++) {
                expandVolume( nAda, nAda.objectpos(gobjects[i]) , nAda.radius(gobjects[i]) );
            }
        }
        
        bool refitVolume(SSBVHNodeAdaptor<ObjectT> nAda)
        {
            if (gobjects.Count == 0) { throw new NotImplementedException(); }  // TODO: fix this... we should never get called in this case...
            
            SSAABB oldbox = box;
            
            computeVolume(nAda);
            if (!box.Equals(oldbox)) {
                if (parent != null) parent.childRefit(nAda);
                return true;
            } else {
                return false;
            }
        }
        
        float SAH(SSAABB box)
        {
            float x_size = box.Max.X - box.Min.X;
            float y_size = box.Max.Y - box.Min.Y;
            float z_size = box.Max.Z - box.Min.Z;
            
            return 2.0f * ( (x_size * y_size) + (x_size * z_size) + (y_size * z_size) );
            
        }
        float SAH(ref SSAABB box)
        {
            float x_size = box.Max.X - box.Min.X;
            float y_size = box.Max.Y - box.Min.Y;
            float z_size = box.Max.Z - box.Min.Z;
            
            return 2.0f * ( (x_size * y_size) + (x_size * z_size) + (y_size * z_size) );
            
        }
        float SAH(BVHNode<ObjectT> * node)
        {
            float x_size = node.box.Max.X - node.box.Min.X;
            float y_size = node.box.Max.Y - node.box.Min.Y;
            float z_size = node.box.Max.Z - node.box.Min.Z;
            
            return 2.0f * ( (x_size * y_size) + (x_size * z_size) + (y_size * z_size) );
        }
        
        float SAH(SSBVHNodeAdaptor<ObjectT> nAda, GO obj)
        {
            float radius = nAda.radius(obj);
            return (float)(4.0 * Math.PI * radius * radius);  // bounding sphere surface area
        }
        
        
        
        SSAABB AABBofPair(ssBVHNode<ObjectT> nodea, ssBVHNode<ObjectT> nodeb)        {
            
            SSAABB box = nodea.box;
            box.ExpandToFit(nodeb.box);
            return box;
        }
        
        float SAHofPair(ssBVHNode<ObjectT> nodea, ssBVHNode<ObjectT> nodeb)
        {
            SSAABB box = nodea.box;
            box.ExpandToFit(nodeb.box);
            return SAH(ref box);
        }
        
        float SAHofPair(SSAABB boxa, SSAABB boxb)
        {
            SSAABB pairbox = boxa;
            pairbox.ExpandToFit(boxb);
            return SAH(ref pairbox);
        }
        
        // The list of all candidate rotations, from "Fast, Effective BVH Updates for Animated Scenes", Figure 1.
        enum Rot
        {
            NONE, L_RL, L_RR, R_LL, R_LR, LL_RR, LL_RL,
        }
        
        class rotOpt : IComparable<rotOpt>
        {  // rotation option
            public float SAH;
            public Rot rot;
            internal rotOpt(float SAH, Rot rot) {
                this.SAH = SAH;
                this.rot = rot;
            }
            public int CompareTo(rotOpt other) {
                return SAH.CompareTo(other.SAH);
            }
        }
        
        List<Rot>_rots = new List<Rot> ((Rot[])Enum.GetValues(typeof(Rot)));
        
        private List<Rot> eachRot
        {
            get {
                return _rots;
            }
        }
        
        /// <summary>
        /// tryRotate looks at all candidate rotations, and executes the rotation with the best resulting SAH (if any)
        /// </summary>
        /// <param name="bvh"></param>
        void tryRotate(ssBVH<ObjectT>& bvh)
        {
            // if we are not a grandparent, then we can't rotate, so queue our parent and bail out
            if (left.isLeaf() && right.isLeaf() )
            {
                if (parent != null)
                {
                    bvh.refitNodes.insert(parent);
                    return;
                }
            }
            
            // for each rotation, check that there are grandchildren as necessary (aka not a leaf)
            // then compute total SAH cost of our branches after the rotation.
            
            float mySAH = SAH(left) + SAH(right);
            
            rotOpt bestRot = eachRot.Min( (rot) => {
                switch (rot) {
                    case Rot.NONE: return new rotOpt(mySAH,Rot.NONE);
                        // child to grandchild rotations
                    case Rot.L_RL:
                        if (right.IsLeaf) return new rotOpt(float.MaxValue,Rot.NONE);
                        else return new rotOpt(SAH(right.left) + SAH(AABBofPair(left,right.right)), rot);
                    case Rot.L_RR:
                        if (right.IsLeaf) return new rotOpt(float.MaxValue,Rot.NONE);
                        else return new rotOpt(SAH(right.right) + SAH(AABBofPair(left,right.left)), rot);
                    case Rot.R_LL:
                        if (left.IsLeaf) return new rotOpt(float.MaxValue,Rot.NONE);
                        else return new rotOpt(SAH(AABBofPair(right,left.right)) + SAH(left.left), rot);
                    case Rot.R_LR:
                        if (left.IsLeaf) return new rotOpt(float.MaxValue,Rot.NONE);
                        else return new rotOpt(SAH(AABBofPair(right,left.left)) + SAH(left.right), rot);
                        // grandchild to grandchild rotations
                    case Rot.LL_RR:
                        if (left.IsLeaf || right.IsLeaf) return new rotOpt(float.MaxValue,Rot.NONE);
                        else return new rotOpt(SAH(AABBofPair(right.right,left.right)) + SAH(AABBofPair(right.left,left.left)), rot);
                    case Rot.LL_RL:
                        if (left.IsLeaf || right.IsLeaf) return new rotOpt(float.MaxValue,Rot.NONE);
                        else return new rotOpt(SAH(AABBofPair(right.left,left.right)) + SAH(AABBofPair(left.left,right.right)), rot);
                        // unknown...
                    default: throw new NotImplementedException("missing implementation for BVH Rotation SAH Computation .. " + rot.ToString());
                }
            });
            
            // perform the best rotation...
            if (bestRot.rot != Rot.NONE) {
                // if the best rotation is no-rotation... we check our parents anyhow..
                if (parent != null) {
                    // but only do it some random percentage of the time.
                    if ((DateTime.Now.Ticks % 100) < 2) {
                        bvh.refitNodes.Add(parent);
                    }
                }
            } else {
                
                if (parent != null) { bvh.refitNodes.Add(parent); }
                
                if ( ((mySAH - bestRot.SAH) / mySAH ) < 0.3f) {
                    return; // the benefit is not worth the cost
                }
                Console.WriteLine("BVH swap {0} from {1} to {2}", bestRot.rot.ToString(), mySAH, bestRot.SAH);
                
                // in order to swap we need to:
                //  1. swap the node locations
                //  2. update the depth (if child-to-grandchild)
                //  3. update the parent pointers
                //  4. refit the boundary box
                ssBVHNode<ObjectT> swap = null;
                switch (bestRot.rot) {
                    case Rot.NONE: break;
                        // child to grandchild rotations
                    case Rot.L_RL: swap = left;  swap.depth++; left  = right.left;  left.parent = this; left.depth--;  right.left  = swap;  swap.parent = right; right.childRefit(nAda); break;
                    case Rot.L_RR: swap = left;  swap.depth++; left  = right.right; left.parent = this; left.depth--;  right.right = swap;  swap.parent = right; right.childRefit(nAda); break;
                    case Rot.R_LL: swap = right; swap.depth++; right =  left.left;  right.parent = this; right.depth--; left.left  = swap;  swap.parent = left;   left.childRefit(nAda); break;
                    case Rot.R_LR: swap = right; swap.depth++; right =  left.right; right.parent = this; right.depth--; left.right = swap;  swap.parent = left;   left.childRefit(nAda); break;
                        
                        // grandchild to grandchild rotations
                    case Rot.LL_RR: swap = left.left; left.left = right.right; right.right = swap; left.left.parent = left; swap.parent = right; left.childRefit(nAda,recurse:false); right.childRefit(nAda); break;
                    case Rot.LL_RL: swap = left.left; left.left = right.left;  right.left  = swap; left.left.parent = left; swap.parent = right; left.childRefit(nAda,recurse:false); right.childRefit(nAda); break;
                        
                        // unknown...
                    default: throw new NotImplementedException("missing implementation for BVH Rotation .. " + bestRot.rot.ToString());
                }
                
            }
            
        }
        
        void splitNode(SSBVHNodeAdaptor<ObjectT> nAda)
        {
            // second, decide which axis to split on, and sort..
            List<ObjectT> splitlist = gobjects;
            
            splitlist.ForEach( o => nAda.unmapObject(o) );
            
            Axis splitAxis = pickSplitAxis();
            switch (splitAxis) // sort along the appropriate axis
            {
                case Axis.X:
                    splitlist.Sort(delegate(GO go1, GO go2) { return nAda.objectpos(go1).X.CompareTo(nAda.objectpos(go2).X); });
                    break;
                case Axis.Y:
                    splitlist.Sort(delegate(GO go1, GO go2) { return nAda.objectpos(go1).Y.CompareTo(nAda.objectpos(go2).Y); });
                    break;
                case Axis.Z:
                    splitlist.Sort(delegate(GO go1, GO go2) { return nAda.objectpos(go1).Z.CompareTo(nAda.objectpos(go2).Z); });
                    break;
                default: throw new NotImplementedException();
            }
            int center = (int) (splitlist.Count / 2); // Find the center object in our current sub-list
            
            gobjects = null;
            
            // create the new left and right nodes...
            left = new ssBVHNode<ObjectT>(nAda.BVH, this, splitlist.GetRange(0,center), splitAxis, this.depth+1); // Split the Hierarchy to the left
            right = new ssBVHNode<ObjectT>(nAda.BVH, this, splitlist.GetRange(center,splitlist.Count - center), splitAxis, this.depth+1); // Split the Hierarchy to the right
            
        }
        
        void splitIfNecessary(SSBVHNodeAdaptor<ObjectT> nAda)
        {
            if (gobjects.Count > nAda.BVH.LEAF_OBJ_MAX)
                splitNode(nAda);
        }
        
        void addObject(SSBVHNodeAdaptor<ObjectT> nAda, GO newOb, ref SSAABB newObBox, float newObSAH)
        {
            if (gobjects != null)
            {
                // add the object and map it to our leaf
                gobjects.Add(newOb);
                nAda.mapObjectToBVHLeaf(newOb,this);
                refitVolume(nAda);
                // split if necessary...
                splitIfNecessary(nAda);
            }
            else
            {
                // find the best way to add this object.. 3 options..
                // 1. send to left node  (L+N,R)
                // 2. send to right node (L,R+N)
                // 3. merge and pushdown left-and-right node (L+R,N)
                
                float leftSAH = SAH(left);
                float rightSAH = SAH(right);
                float sendLeftSAH = rightSAH + SAH(left.box.ExpandedBy(newObBox));    // (L+N,R)
                float sendRightSAH = leftSAH + SAH(right.box.ExpandedBy(newObBox));   // (L,R+N)
                float mergedLeftAndRightSAH = SAH(AABBofPair(left,right)) + newObSAH; // (L+R,N)
                
                // Doing a merge-and-pushdown can be expensive, so we only do it if it's notably better
                const float MERGE_DISCOUNT = 0.3f;
                
                if (mergedLeftAndRightSAH < ( Math.Min(sendLeftSAH,sendRightSAH)) * MERGE_DISCOUNT )
                {
                    // merge and pushdown left and right as a new node..
                    var mSubnode = new ssBVHNode<ObjectT>(nAda.BVH);
                    mSubnode.left = left;
                    mSubnode.right = right;
                    mSubnode.parent = this;
                    mSubnode.gobjects = null; // we need to be an interior node... so null out our object list..
                    left.parent = mSubnode;
                    right.parent = mSubnode;
                    mSubnode.childRefit(nAda, recurse:false);
                    
                    // make new subnode for obj
                    var nSubnode = new ssBVHNode<ObjectT>(nAda.BVH);
                    nSubnode.parent = this;
                    nSubnode.gobjects = new List<ObjectT>{ newOb };
                    nAda.mapObjectToBVHLeaf(newOb,nSubnode);
                    nSubnode.computeVolume(nAda);
                    
                    // make assignments..
                    this.left = mSubnode;
                    this.right = nSubnode;
                    this.setDepth(this.depth); // propagate new depths to our children.
                    this.childRefit(nAda);
                }
                else
                {
                    if ( sendLeftSAH < sendRightSAH )
                        // send left
                        left.addObject(nAda,newOb,ref newObBox, newObSAH);
                    else
                        // send right
                        right.addObject(nAda,newOb,ref newObBox, newObSAH);
                }
            }
            
        }
        
        int countBVHNodes()
        {
            if (gobjects != null) {
                return 1;
            } else {
                return left.countBVHNodes() + right.countBVHNodes();
            }
        }
        
        void removeObject(SSBVHNodeAdaptor<ObjectT> nAda, GO newOb) {
            if (gobjects == null) { throw new Exception("removeObject() called on nonLeaf!"); }
            
            nAda.unmapObject(newOb);
            gobjects.Remove(newOb);
            if (gobjects.Count > 0) {
                refitVolume(nAda);
            } else {
                // our leaf is empty, so collapse it if we are not the root...
                if (parent != null) {
                    gobjects = null;
                    parent.removeLeaf(nAda, this);
                    parent = null;
                }
            }
        }
        
        setDepth(int newdepth)
        {
            this.depth = newdepth;
            if (gobjects == null) {
                left.setDepth(newdepth+1);
                right.setDepth(newdepth+1);
            }
        }
        
        void removeLeaf(BVHNode * leafToRemove)
        {
            if (left == null || right == null) { throw new Exception("bad intermediate node"); }
            ssBVHNode<ObjectT> keepLeaf;
            
            if( leafToRemove == left )
                keepLeaf = right;
            else
                keepLeaf = left;

            
            // "become" the leaf we are keeping.
            box = keepLeaf.box;
            left = keepLeaf.left;
            right = keepLeaf.right;
            objects = keepLeaf.gobjects;
            
            // clear the leaf..
            // keepLeaf.left = null; keepLeaf.right = null; keepLeaf.gobjects = null; keepLeaf.parent = null;
            
            if (gobjects == null)
            {
                left.parent = this;
                right.parent = this;  // reassign child parents..
                
                setDepth(depth); // recompute children depth
            }
            else
                // map the objects we adopted to us...
                gobjects.ForEach( o => { nAda.mapObjectToBVHLeaf(o,this); } );
            
            // propagate our new volume..
            if (parent != null) {
                parent.childRefit(nAda);
            }
        }
        
        
        ssBVHNode<ObjectT> rootNode()
        {
            ssBVHNode<ObjectT> cur = this;
            while (cur.parent != null) { cur = cur.parent; }
            return cur;
        }
        
        
        void findOverlappingLeaves(SSBVHNodeAdaptor<ObjectT> nAda, Vector3 origin, float radius, List<ssBVHNode<ObjectT>> overlapList)
        {
            if (toAABB().IntersectsSphere(origin,radius)) {
                if (gobjects != null) {
                    overlapList.Add(this);
                } else {
                    left.findOverlappingLeaves(nAda,origin,radius,overlapList);
                    right.findOverlappingLeaves(nAda,origin,radius,overlapList);
                }
            }
        }
        
        void findOverlappingLeaves(SSBVHNodeAdaptor<ObjectT> nAda, SSAABB aabb, List<ssBVHNode<ObjectT>> overlapList)
        {
            if (toAABB().IntersectsAABB(aabb)) {
                if (gobjects != null) {
                    overlapList.Add(this);
                } else {
                    left.findOverlappingLeaves(nAda,aabb,overlapList);
                    right.findOverlappingLeaves(nAda,aabb,overlapList);
                }
            }
        }
        
        void childExpanded(SSBVHNodeAdaptor<ObjectT> nAda, ssBVHNode<ObjectT> child)
        {
            bool expanded = false;
            
            if (child.box.Min.X < box.Min.X) {
                box.Min.X = child.box.Min.X; expanded = true;
            }
            if (child.box.Max.X > box.Max.X) {
                box.Max.X = child.box.Max.X; expanded = true;
            }
            if (child.box.Min.Y < box.Min.Y) {
                box.Min.Y = child.box.Min.Y; expanded = true;
            }
            if (child.box.Max.Y > box.Max.Y) {
                box.Max.Y = child.box.Max.Y; expanded = true;
            }
            if (child.box.Min.Z < box.Min.Z) {
                box.Min.Z = child.box.Min.Z; expanded = true;
            }
            if (child.box.Max.Z > box.Max.Z) {
                box.Max.Z = child.box.Max.Z; expanded = true;
            }
            
            if (expanded && parent != null) {
                parent.childExpanded(nAda, this);
            }
        }
        
        void childRefit(SSBVHNodeAdaptor<ObjectT> nAda, bool recurse=true)
        {
            SSAABB oldbox = box;
            box.Min.X = left.box.Min.X; box.Max.X = left.box.Max.X;
            box.Min.Y = left.box.Min.Y; box.Max.Y = left.box.Max.Y;
            box.Min.Z = left.box.Min.Z; box.Max.Z = left.box.Max.Z;
            
            if (right.box.Min.X < box.Min.X) { box.Min.X = right.box.Min.X; }
            if (right.box.Min.Y < box.Min.Y) { box.Min.Y = right.box.Min.Y; }
            if (right.box.Min.Z < box.Min.Z) { box.Min.Z = right.box.Min.Z; }
            
            if (right.box.Max.X > box.Max.X) { box.Max.X = right.box.Max.X; }
            if (right.box.Max.Y > box.Max.Y) { box.Max.Y = right.box.Max.Y; }
            if (right.box.Max.Z > box.Max.Z) { box.Max.Z = right.box.Max.Z; }
            
            if (recurse && parent != null) { parent.childRefit(nAda); }
        }
        
 
        
    }
}
