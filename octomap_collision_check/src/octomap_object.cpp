#include <octomap_collision_check/octomap_object.h>

namespace octomap {

	OctomapObject::OctomapObject()
	 : m_ocTree(NULL), parent(NULL)
	{

	}

	OctomapObject::OctomapObject(OcTree* tree)
	 : m_ocTree(tree), parent(NULL)
	{

	}

	OctomapObject::~OctomapObject(){
		if (m_ocTree)
			delete m_ocTree;

		// delete all children
		for (std::vector<OctomapObject*>::iterator it = children.begin(); it != children.end(); ++it) {
			delete (*it);
		}
		children.clear();

	}

	void OctomapObject::addChild(OctomapObject* child){
		child->setParent(this);
		children.push_back(child);
	}

	void OctomapObject::init(){
		MotionRange r;
		r.min = r.range = r.current = 0.0;
		motionRanges.resize(numDOF(), r);

	}

	void OctomapObject::move(const std::vector<double>& param){
		unsigned dof = numDOF();
		assert(param.size() == dof);
		assert(motionRanges.size() == dof);

		// TODO check range?
		for (unsigned i = 0; i < dof; ++i){
			motionRanges[i].current = param[i];
			motionRanges[i].current = std::min(motionRanges[i].current, 1.0);
		}

		mapParams();
	}

	void OctomapObject::moveDiff(const std::vector<double>& param){
		unsigned dof = numDOF();
		assert(param.size() == dof);
		assert(motionRanges.size() == dof);

		for (unsigned i = 0; i < dof; ++i){
			motionRanges[i].current += param[i];
			motionRanges[i].current = std::min(motionRanges[i].current, 1.0);
		}

		mapParams();
	}

	void OctomapObject::setRange(unsigned dofIdx, double min, double max){
		assert (dofIdx < numDOF());

		motionRanges.at(dofIdx).min = min;
		motionRanges.at(dofIdx).range = max-min;
	}

	void StaticObject::mapParams(){
		return;
	}

	void PlanarObject::mapParams(){
		origin.trans().x() = mapRange(0);
		origin.trans().y() = mapRange(1);
		double yaw = mapRange(2);
		origin.rot() = octomath::Quaternion(0.0, 0.0, yaw);

	}


	void PrismaticObject::mapParams(){
		origin.trans().x() = mapRange(0);
	}






    
} //namespace
