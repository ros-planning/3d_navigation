/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __ENVIRONMENT_3DCOLLISIONS_H_
#define __ENVIRONMENT_3DCOLLISIONS_H_

#include <pcl_ros/point_cloud.h>
#include <pcl/surface/concave_hull.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

//eight-connected grid
#define NAVXYTHETALAT_DXYWIDTH 8

#define ENVNAVXYTHETALAT_DEFAULTOBSTHRESH 254	//see explanation of the value below

#define SBPL_XYTHETALAT_MAXSTATESFORLOOKUP 100000000 //maximum number of states for storing them into lookup (as opposed to hash)

//definition of theta orientations
//0 - is aligned with X-axis in the positive direction (1,0 in polar coordinates)
//theta increases as we go counterclockwise
//number of theta values - should be power of 2
#define NAVXYTHETALAT_THETADIRS 16

//number of actions per x,y,theta state
#define NAVXYTHETALAT_DEFAULT_ACTIONWIDTH 5 //decrease, increase, same angle while moving plus decrease, increase angle while standing.

#define NAVXYTHETALAT_COSTMULT_MTOMM 1000
// already defined, no need to change?
//
//typedef struct{
//	double x;
//	double y;
//} EnvNAVXYTHETALAT2Dpt_t;
//
//typedef struct{
//	double x;
//	double y;
//	double theta;
//} EnvNAVXYTHETALAT3Dpt_t;
//
//
//typedef struct EnvNAVXYTHETALAT3DCELL{
//	int x;
//	int y;
//	int theta;
//	int iteration;
//public:
//	bool operator == (EnvNAVXYTHETALAT3DCELL cell) {return (x==cell.x && y==cell.y && theta==cell.theta);}
//} EnvNAVXYTHETALAT3Dcell_t;
//
//
//typedef struct
//{
//	unsigned char aind; //index of the action (unique for given starttheta)
//	char starttheta;
//	char dX;
//	char dY;
//	char endtheta;
//	unsigned int cost;
//	vector<sbpl_2Dcell_t> intersectingcellsV;
//	//start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
//	vector<EnvNAVXYTHETALAT3Dpt_t> intermptV;
//	//start at 0,0,starttheta and end at endcell in discrete domain
//	vector<EnvNAVXYTHETALAT3Dcell_t> interm3DcellsV;
//} EnvNAVXYTHETALATAction_t;
//
//
//typedef struct
//{
//	int stateID;
//	int X;
//	int Y;
//	char Theta;
//	int iteration;
//} EnvNAVXYTHETALATHashEntry_t;
//
//
//typedef struct
//{
//	int motprimID;
//	unsigned char starttheta_c;
//	int additionalactioncostmult;
//	EnvNAVXYTHETALAT3Dcell_t endcell;
//	//intermptV start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
//	vector<EnvNAVXYTHETALAT3Dpt_t> intermptV;
//}SBPL_xytheta_mprimitive;
//
//
////variables that dynamically change (e.g., array of states, ...)
//typedef struct
//{
//
//	int startstateid;
//	int goalstateid;
//
//	bool bInitialized;
//
//	//any additional variables
//
//
//}EnvironmentNav3DCollisions_t;
//
//configuration parameters
typedef struct ENV_NAV3DCOLL_CONFIG
{
	int EnvWidth_c;
	int EnvHeight_c;
	int StartX_c;
	int StartY_c;
	int StartTheta;
	int EndX_c;
	int EndY_c;
	int EndTheta;
	unsigned char** Grid2D;

	//the value at which and above which cells are obstacles in the maps sent from outside
	//the default is defined above
	unsigned char obsthresh;

	//the value at which and above which until obsthresh (not including it) cells have the nearest obstacle at distance smaller than or equal to
	//the inner circle of the robot. In other words, the robot is definitely colliding with the obstacle, independently of its orientation
	//if no such cost is known, then it should be set to obsthresh (if center of the robot collides with obstacle, then the whole robot collides with it
	//independently of its rotation)
	unsigned char cost_inscribed_thresh;

	//the value at which and above which until cost_inscribed_thresh (not including it) cells
	//**may** have a nearest osbtacle within the distance that is in between the robot inner circle and the robot outer circle
	//any cost below this value means that the robot will NOT collide with any obstacle, independently of its orientation
	//if no such cost is known, then it should be set to 0 or -1 (then no cell cost will be lower than it, and therefore the robot's footprint will always be checked)
	int cost_possibly_circumscribed_thresh; //it has to be integer, because -1 means that it is not provided.

	double nominalvel_mpersecs;
	double timetoturn45degsinplace_secs;
	double costmap_origin_x;
	double costmap_origin_y;
	double cellsize_m;

	int dXY[NAVXYTHETALAT_DXYWIDTH][2];

	EnvNAVXYTHETALATAction_t** ActionsV; //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
	vector<EnvNAVXYTHETALATAction_t*>* PredActionsV; //PredActionsV[i] - vector of pointers to the actions that result in a state with theta = i

	int actionwidth; //number of motion primitives
	vector<SBPL_xytheta_mprimitive> mprimV;

	vector<sbpl_2Dpt_t> FootprintPolygon;
} EnvNav3DCollisionsConfig_t;



class SBPL2DGridSearch;

/** \brief 3D (x,y,theta) planning using lattice-based graph problem. For general structure see comments on parent class DiscreteSpaceInformation
For info on lattice-based planning used here, you can check out the paper: 
Maxim Likhachev and Dave Ferguson, " Planning Long Dynamically-Feasible Maneuvers for Autonomous Vehicles", IJRR'09
*/
class EnvironmentNav3DCollisionsBase : public DiscreteSpaceInformation
{

public:

	EnvironmentNav3DCollisionsBase( planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor );

  /**
	 * \brief way to set up various parameters. For a list of parameters, see the body of the function - it is pretty straightforward
  */
	virtual bool SetEnvParameter(const char* parameter, int value);
  /**
	 * \brief returns the value of specific parameter - see function body for the list of parameters
  */
	virtual int GetEnvParameter(const char* parameter);
  /**
	 * \brief see comments on the same function in the parent class
   */
	bool InitializeMDPCfg(MDPConfig *MDPCfg);

	virtual bool InitializeEnv(const char* sEnvFile){return false;};
  /**
	 * \brief see comments on the same function in the parent class
   */
	virtual int  GetFromToHeuristic(int FromStateID, int ToStateID) = 0;
  /**
	 * \brief see comments on the same function in the parent class
   */
	virtual int  GetGoalHeuristic(int stateID) = 0;
  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual int  GetStartHeuristic(int stateID) = 0;
  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) = 0;
  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual void SetAllPreds(CMDPSTATE* state);
  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) = 0;

  /**
	 * \brief see comments on the same function in the parent class
  */
	virtual void EnsureHeuristicsUpdated(bool bGoalHeuristics); 

  /**
	 * \brief see comments on the same function in the parent class
  */
	void PrintEnv_Config(FILE* fOut);

  /**
	 * \brief initialize environment. Gridworld is defined as matrix A of size width by height. 
   * So, internally, it is accessed as A[x][y] with x ranging from 0 to width-1 and and y from 0 to height-1
   * Each element in A[x][y] is unsigned char. A[x][y] = 0 corresponds to fully traversable and cost is just Euclidean distance
   * The cost of transition between two neighboring cells is EuclideanDistance*(max(A[sourcex][sourcey],A[targetx][targety])+1)
   * f A[x][y] >= obsthresh, then in the above equation it is assumed to be infinite.
   * The cost also incorporates the length of a motion primitive and its cost_multiplier (see getcost function)
   * mapdata is a pointer to the values of A. If it is null, then A is initialized to all zeros. Mapping is: A[x][y] = mapdata[x+y*width]
   * start/goal are given by startx, starty, starttheta, goalx,goaly, goaltheta in meters/radians. 
   * If they are not known yet, just set them to 0. Later setgoal/setstart can be executed
   * finally obsthresh defined obstacle threshold, as mentioned above
   * goaltolerances are currently ignored
   * for explanation of perimeter, see comments for InitializeEnv function that reads all from file
   * cellsize is discretization in meters
   * nominalvel_mpersecs is assumed velocity of vehicle while moving forward in m/sec
   * timetoturn45degsinplace_secs is rotational velocity in secs/45 degrees turn
  */
    bool InitializeEnv(int width, int height,
			double origin_x, double origin_y,
			double res, const unsigned char* mapdata,
			const std::vector<std::string>& perimeterptsV, const motion_planning_msgs::RobotState& robot_state,
					   double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
					   unsigned char obsthresh, const char* sMotPrimFile, vector<sbpl_2Dpt_t> base_fp, vector<sbpl_2Dpt_t>* new_fp);

  /**
	 * \brief update the traversability of a cell<x,y>
  */
    bool UpdateCost(int x, int y, unsigned char newcost);

  /**
   * \brief re-setting the whole 2D map
   * transform from linear array mapdata to the 2D matrix used internally: Grid2D[x][y] = mapdata[x+y*width]
  */
	bool SetMap(const unsigned char* mapdata);


  /**
   * \brief this function fill in Predecessor/Successor states of edges whose costs changed
   * It takes in an array of cells whose traversability changed, and returns (in vector preds_of_changededgesIDV) 
   * the IDs of all states that have outgoing edges that go through the changed cells
  */
	virtual void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV) = 0;
  /**
	 * \brief same as GetPredsofChangedEdges, but returns successor states. Both functions need to be present for incremental search
  */
	virtual void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV) = 0;

  /**
	 * returns true if cell is untraversable
  */
	bool IsObstacle(int x, int y);
  /**
   * \brief returns false if robot intersects obstacles or lies outside of the map. Note this is pretty expensive operation since it computes the footprint
   * of the robot based on its x,y,theta
  */
	virtual bool IsValidConfiguration(int X, int Y, int Theta);

	/// Check for 3D collision of the robot's kinematic state.
	/// x,y,theta are in continuous costmap coords, will be transformed
	/// into the 3D coll. map coords.
	bool isIn3DCollision(double x, double y, double theta);

  /**
	 * \brief returns environment parameters. Useful for creating a copy environment
  */
	void GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double* starttheta, double* goalx, double* goaly, double* goaltheta,
			double* cellsize_m, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs, unsigned char* obsthresh, vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
  /**
	 * \brief get internal configuration data structure
  */
	const EnvNav3DCollisionsConfig_t* GetEnvNavConfig();


  virtual ~EnvironmentNav3DCollisionsBase();
  /**
   * \brief prints time statistics
  */
  void PrintTimeStat(FILE* fOut);
  /** 
   * \brief returns the cost corresponding to the cell <x,y>
  */
	unsigned char GetMapCost(int x, int y);

  /**
	 * \brief returns true if cell is within map
  */
	inline bool IsWithinMapCell(int X, int Y) const{
		return (X >= 0 && X < EnvNAVXYTHETALATCfg.EnvWidth_c &&
			Y >= 0 && Y < EnvNAVXYTHETALATCfg.EnvHeight_c);
	}

	inline bool IsOccupied(int x, int y) const{
		return EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.obsthresh;
	}

	virtual inline bool IsValidCell(int x, int y) const{
		return (IsWithinMapCell(x, y) && !IsOccupied(x, y));
	}

  
  /** \brief Transform a pose into discretized form. The angle 'pth' is
      considered to be valid if it lies between -2pi and 2pi (some
      people will prefer 0<=pth<2pi, others -pi<pth<=pi, so this
      compromise should suit everyone).
      
      \note Even if this method returns false, you can still use the
      computed indices, for example to figure out how big your map
      should have been.
      
      \return true if the resulting indices lie within the grid bounds
      and the angle was valid.
  */
  bool PoseContToDisc(double px, double py, double pth,
		      int &ix, int &iy, int &ith) const;
  
  /** \brief Transform grid indices into a continuous pose. The computed
      angle lies within 0<=pth<2pi.
      
      \note Even if this method returns false, you can still use the
      computed indices, for example to figure out poses that lie
      outside of your current map.
      
      \return true if all the indices are within grid bounds.
  */
  bool PoseDiscToCont(int ix, int iy, int ith,
		      double &px, double &py, double &pth) const;

  /** \brief prints environment variables for debugging
    */
  virtual void PrintVars(){};

  /** @brief Recompute the robot's footprint based on kinematic state,
   * also recomputes all motion primitive footprints.
   *
   * Uses downprojectShape(), so currently only supports MESH shape
   * types. */
  bool updateFootprint();

  /** @brief Project points from the given shape at global pose
   * transform onto z=0 plane, divide by the cell size, discretize
   * them, and append them to the output vector "points".
   *
   * Currently only supports MESH shape types, and silently ignores
   * others. */
  void downprojectShape( const shapes::ShapeConstPtr shape,
                         const Eigen::Affine3d& transform,
                         std::vector<cv::Point>& points ) const;

  /** @brief Notify this environment that a collision update has been
   * received by the planning_scene_monitor_. */
  void notifyCollisionReceived();

  /** @brief Set kinematic_state_ = robot_state, update
   * downprojectedFootprint from it, copy downprojectedFootprint into
   * new_fp, and return true if the footprint has changed. */
  bool updateKinematicState(const robot_state::RobotState &robot_state, vector<sbpl_2Dpt_t>* new_fp);

///// Seems like MoveIt makes this unnecessary...
/////  void updateAttachedObjects(const mapping_msgs::AttachedCollisionObject& coll);

  /// sets the robot's kinematic state to coordinate x,y,theta (2D costmap coordinates)
  void updateRobotPosition(double x, double y, double theta);

  unsigned getNum3DChecks(){ return m_num3DCollChecks;};
  unsigned getNum2DChecks(){ return m_num2DCollChecks;};

  void printCollisionStats(){
    printf("Succs:%d 1D:%d 2D:%d 3D:%d\n",succ_count,center_count,footprint_count,mesh_count);
  }

  void resetCollisionCount(){m_num3DCollChecks=0;m_num2DCollChecks=0;};
  bool use_multi_layer;

 protected:

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d_collisions;
  bool always3Dcheck;
  double basketRadius;
  void printFootprint(char* filename, vector<sbpl_2Dcell_t> footprint);
  virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action, bool* possible_collision);
  int succ_count;
  int center_count;
  int footprint_count;
  int mesh_count;

  inline int thetaCont2Disc(double theta_rad){
	 return ContTheta2Disc(theta_rad, NAVXYTHETALAT_THETADIRS);
  }
  inline double thetaDisc2Cont(int theta_disc){
	  return DiscTheta2Cont(theta_disc, NAVXYTHETALAT_THETADIRS);
  }

  inline int xyCont2Disc(double x){
	 return CONTXY2DISC(x, EnvNAVXYTHETALATCfg.cellsize_m);
  }

  inline double xyDisc2Cont(int x_disc){
	  return DISCXY2CONT(x_disc, EnvNAVXYTHETALATCfg.cellsize_m);
  }

  void visualize3DCollsisions();

	//member data
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_state::RobotState kinematic_state_;
	unsigned m_num3DCollChecks;
	unsigned m_num2DCollChecks;
  unsigned firstSol2DChecks;
  unsigned firstSol3DChecks;
	std::vector<std::string> m_footprintLinkNames;
	EnvNav3DCollisionsConfig_t EnvNAVXYTHETALATCfg;
	vector<EnvNAVXYTHETALAT3Dcell_t> affectedsuccstatesV; //arrays of states whose outgoing actions cross cell 0,0
	vector<EnvNAVXYTHETALAT3Dcell_t> affectedpredstatesV; //arrays of states whose incoming actions cross cell 0,0
	int iteration;
	int startstateid;
	int goalstateid;

	bool bInitialized;

	ros::Time m_collisionObjectsTime;
  vector<sbpl_2Dpt_t> downprojectedFootprint;

	//2D search for heuristic computations
	bool bNeedtoRecomputeStartHeuristics; //set whenever grid2Dsearchfromstart needs to be re-executed
	bool bNeedtoRecomputeGoalHeuristics; //set whenever grid2Dsearchfromgoal needs to be re-executed
	SBPL2DGridSearch* grid2Dsearchfromstart; //computes h-values that estimate distances from start x,y to all cells
	SBPL2DGridSearch* grid2Dsearchfromgoal;  //computes h-values that estimate distances to goal x,y from all cells

	ros::Publisher m_collisionMarkerPub, m_footprintPolygonPub, footPointsPub;

 	virtual void ReadConfiguration(FILE* fCfg) {};

	void InitializeEnvConfig(vector<SBPL_xytheta_mprimitive>* motionprimitiveV);


	bool CheckQuant(FILE* fOut);

	void SetConfiguration(int width, int height,
			      /** if mapdata is NULL the grid is initialized to all freespace */
			      const unsigned char* mapdata,
			      int startx, int starty, int starttheta,
			      int goalx, int goaly, int goaltheta,
				  double origin_x, double origin_y,
				  double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs);
	
	bool InitGeneral( vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
	void PrecomputeActionswithBaseMotionPrimitive(vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
	void PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xytheta_mprimitive>* motionprimitiveV);
	void PrecomputeActions();

	void CreateStartandGoalStates();

	virtual void InitializeEnvironment() = 0;

	void ComputeHeuristicValues();


  bool printFP;
	void CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint);
	void CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, set<pair<int,int> >* footprint);
	void CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, set<pair<int,int> >* footprint, const vector<sbpl_2Dpt_t>& FootprintPolygon);
	void RemoveSourceFootprint(EnvNAVXYTHETALAT3Dpt_t sourcepose, vector<sbpl_2Dcell_t>* footprint);
	void RemoveSourceFootprint(EnvNAVXYTHETALAT3Dpt_t sourcepose, vector<sbpl_2Dcell_t>* footprint, const vector<sbpl_2Dpt_t>& FootprintPolygon);

	virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETALATAction_t*>* actionindV=NULL) = 0;

	double EuclideanDistance_m(int X1, int Y1, int X2, int Y2);

	void ComputeReplanningData();
	void ComputeReplanningDataforAction(EnvNAVXYTHETALATAction_t* action);

	bool ReadMotionPrimitives(FILE* fMotPrims);
	bool ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn);
	bool ReadinCell(EnvNAVXYTHETALAT3Dcell_t* cell, FILE* fIn);
	bool ReadinPose(EnvNAVXYTHETALAT3Dpt_t* pose, FILE* fIn);

	void PrintHeuristicValues();

  void getFootprintCells(sbpl_2Dcell_t start, vector<sbpl_2Dpt_t>* polygon, set<pair<int,int> >* footprint);

  //FIFO class
  class FIFO{
    public:
      FIFO(int length) : size(length){
        head = 0;
        tail = 0;
        q = new sbpl_2Dcell_t[size];
      };

      ~FIFO(){
        delete [] q;
      }

      void clear(){
        head = 0;
        tail = 0;
      }

      bool empty(){
        return head == tail;
      }

      bool insert(int x, int y){
        int temp = tail;
        if(head==temp-1 || (temp==0 && head==size-1)){
          printf("FIFO FULL!\n");
          return false;
        }
        q[head].x = x;
        q[head].y = y;
        head++;
        if(head == size)
          head = 0;
        return true;
      }

      bool remove(int* x, int* y){
        if(head==tail)
          return false;
        *x = q[tail].x;
        *y = q[tail].y;
        tail++;
        if(tail == size)
          tail = 0;
        return true;
      }

    private:
      int head;
      int tail;
      int size;
      sbpl_2Dcell_t* q;
  };

  FIFO* footprint_fifo;
};


class EnvironmentNav3DCollisions : public EnvironmentNav3DCollisionsBase
{

 public:
  EnvironmentNav3DCollisions( planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor )
    : EnvironmentNav3DCollisionsBase( planning_scene_monitor )
  {
	HashTableSize = 0;
	Coord2StateIDHashTable = NULL;
	Coord2StateIDHashTable_lookup = NULL; 
  };

  ~EnvironmentNav3DCollisions();

  /** \brief sets start in meters/radians
    */
  int SetStart(double x, double y, double theta);
  /** \brief sets goal in meters/radians
  */
  int SetGoal(double x, double y, double theta);
  void drawPose(double x, double y, double theta, int i);
  /** \brief sets goal tolerance. (Note goal tolerance is ignored currently)
    */
  void SetGoalTolerance(double tol_x, double tol_y, double tol_theta) { /**< not used yet */ }
  /** \brief returns state coordinates of state with ID=stateID
    */
  void GetCoordFromState(int stateID, int& x, int& y, int& theta) const;
  /** \brief returns stateID for a state with coords x,y,theta
    */
  int GetStateFromCoord(int x, int y, int theta);

  /** \brief converts a path given by stateIDs into a sequence of coordinates. Note that since motion primitives are short actions represented as a sequence of points,
  the path returned by this function contains much more points than the number of points in the input path. The returned coordinates are in meters,meters,radians
  */
  void ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<EnvNAVXYTHETALAT3Dpt_t>* xythetaPath); 
  virtual void drawStuff(EnvNAVXYTHETALATAction_t* action,int x,int y) = 0;
  /** \brief prints state info (coordinates) into file
    */
  void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
  /** \brief returns all predecessors states and corresponding costs of actions
    */
  virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
  /** \brief returns all successors states, costs of corresponding actions and pointers to corresponding actions, each of which is a motion primitive
  if actionindV is NULL, then pointers to actions are not returned
  */
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETALATAction_t*>* actionindV=NULL);

  /** \brief this function fill in Predecessor/Successor states of edges whose costs changed
  It takes in an array of cells whose traversability changed, and returns (in vector preds_of_changededgesIDV) 
  the IDs of all states that have outgoing edges that go through the changed cells
  */
  void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV);
  /** \brief same as GetPredsofChangedEdges, but returns successor states. Both functions need to be present for incremental search
    */
  void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV);

  /** \brief see comments on the same function in the parent class
    */
  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

  /** \brief see comments on the same function in the parent class
    */
  virtual int  GetFromToHeuristic(int FromStateID, int ToStateID);
  /** \brief see comments on the same function in the parent class
    */
  virtual int  GetGoalHeuristic(int stateID);
  /** \brief see comments on the same function in the parent class
    */
  virtual int  GetStartHeuristic(int stateID);

  /** \brief see comments on the same function in the parent class
    */
  virtual int	 SizeofCreatedEnv();
  /** \brief see comments on the same function in the parent class
    */
  virtual void PrintVars(){};


 protected:
  //hash table of size x_size*y_size. Maps from coords to stateId	
  int HashTableSize;
  vector<EnvNAVXYTHETALATHashEntry_t*>* Coord2StateIDHashTable;
  //vector that maps from stateID to coords	
  vector<EnvNAVXYTHETALATHashEntry_t*> StateID2CoordTable;
  
  EnvNAVXYTHETALATHashEntry_t** Coord2StateIDHashTable_lookup; 

  unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta);

  EnvNAVXYTHETALATHashEntry_t* GetHashEntry_hash(int X, int Y, int Theta);
  EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta);
  EnvNAVXYTHETALATHashEntry_t* GetHashEntry_lookup(int X, int Y, int Theta);
  EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta);

  //pointers to functions
  EnvNAVXYTHETALATHashEntry_t* (EnvironmentNav3DCollisions::*GetHashEntry)(int X, int Y, int Theta);
  EnvNAVXYTHETALATHashEntry_t* (EnvironmentNav3DCollisions::*CreateNewHashEntry)(int X, int Y, int Theta);


  virtual void InitializeEnvironment();

  void PrintHashTableHist(FILE* fOut);



};


#endif

