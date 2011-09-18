#ifndef OCTOMAP_OBJECT_H
#define OCTOMAP_OBJECT_H

#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "bounding_box.h"

namespace octomap {

	class OctomapObject{
	public:
		OctomapObject();
		OctomapObject(OcTree* tree);
		virtual ~OctomapObject();
		void setOcTree(OcTree* tree){ m_ocTree = tree; }
		void setParent(OctomapObject* parent) {parent = parent;}
		const OcTree* getOcTree() const {return m_ocTree; }
		void addChild(OctomapObject* child);
	    inline void setOrigin(const octomap::pose6d& o) { origin = o; }
	    inline octomap::pose6d&  getOrigin() { return origin; }
	    inline octomap::pose6d  getOrigin() const { return origin; }

	    virtual unsigned numDOF() = 0;
	    //virtual void setRange() = 0;
	    virtual void init();
	    void move(const std::vector<double>& param);
	    void moveDiff(const std::vector<double>& param);
	    virtual void setRange(unsigned dofIdx, double min, double max);

	    typedef std::vector<OctomapObject*>::iterator iterator;
	    typedef std::vector<OctomapObject*>::const_iterator const_iterator;
	    inline iterator begin () { return (children.begin ()); }
	    inline iterator end ()   { return (children.end ()); }
	    inline const_iterator begin () const { return children.begin (); }
	    inline const_iterator end () const  { return children.end (); }
	    inline size_t size () const { return children.size (); }



	protected:
	    struct MotionRange{
	    	double min;
	    	double range;
	    	double current;
	    };
	    virtual void mapParams() = 0;
	    inline double mapRange (unsigned i) const{
	    	return motionRanges[i].min + motionRanges[i].range * motionRanges[i].current;
	    }
		OcTree* m_ocTree;
		octomap::pose6d origin;
		OctomapObject* parent;
	    std::vector<OctomapObject*> children;
	    /// motion range stored as min + total range
	    std::vector<MotionRange> motionRanges;

	};

	class StaticObject : public OctomapObject{
	public:
		StaticObject() : OctomapObject() {}
		StaticObject(OcTree* tree) : OctomapObject(tree){}
		virtual ~StaticObject() {}

		virtual unsigned numDOF() {return 0;}

	protected:
		virtual void mapParams();


	};

	class PlanarObject : public OctomapObject{
	public:
		PlanarObject() : OctomapObject() {}
		PlanarObject(OcTree* tree) : OctomapObject(tree){}
		virtual ~PlanarObject() {}

		virtual unsigned numDOF() {return 3;}

	protected:
		virtual void mapParams();
	};

	class PrismaticObject : public OctomapObject{
	public:
		PrismaticObject() : OctomapObject() {}
		PrismaticObject(OcTree* tree) : OctomapObject(tree){}
		virtual ~PrismaticObject() {}

		virtual unsigned numDOF() {return 1;}

	protected:
		virtual void mapParams();
	};




} // namespace 

#endif 
