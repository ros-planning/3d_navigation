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
#include <iostream>
using namespace std;

#include <sbpl/headers.h>
#include <sbpl_lattice_planner_layer_3d/environment_nav_3d_collisions.h>
#include <planning_environment/models/model_utils.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/surface/concave_hull.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PolygonStamped.h>
#include <mapping_msgs/CollisionObjectOperation.h>

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

#define XYTHETA2INDEX(X,Y,THETA) (THETA + X*NAVXYTHETALAT_THETADIRS + Y*EnvNAVXYTHETALATCfg.EnvWidth_c*NAVXYTHETALAT_THETADIRS)

//-----------------constructors/destructors-------------------------------
EnvironmentNav3DCollisionsBase::EnvironmentNav3DCollisionsBase()
 : m_planningCollisionModel("robot_description"),
   m_kinematicState(NULL),
   m_num3DCollChecks(0), m_num2DCollChecks(0),
   cloud_3d_collisions(new pcl::PointCloud<pcl::PointXYZ>)
{
  always3Dcheck = false;
	EnvNAVXYTHETALATCfg.obsthresh = ENVNAVXYTHETALAT_DEFAULTOBSTHRESH;
	EnvNAVXYTHETALATCfg.cost_inscribed_thresh = EnvNAVXYTHETALATCfg.obsthresh; //the value that pretty much makes it disabled
	EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = -1; //the value that pretty much makes it disabled

	grid2Dsearchfromstart = NULL;
	grid2Dsearchfromgoal = NULL;
	bNeedtoRecomputeStartHeuristics = true;
	bNeedtoRecomputeGoalHeuristics = true;
	iteration = 0;

	m_kinematicState = new planning_models::KinematicState(m_planningCollisionModel.getKinematicModel());
	bInitialized = false;
	m_collisionReceived = false;

	EnvNAVXYTHETALATCfg.actionwidth = NAVXYTHETALAT_DEFAULT_ACTIONWIDTH;

	//no memory allocated in cfg yet
	EnvNAVXYTHETALATCfg.Grid2D = NULL;
	EnvNAVXYTHETALATCfg.ActionsV = NULL;
	EnvNAVXYTHETALATCfg.PredActionsV = NULL;

	ros::NodeHandle nh("~");
	m_collisionMarkerPub = nh.advertise<visualization_msgs::MarkerArray>("planning_visualizer_array", 10);
	m_footprintPolygonPub = nh.advertise<geometry_msgs::PolygonStamped>("downprojected_footprint", 10);
  footPointsPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("cool_stuff",10);

  cloud_3d_collisions->header.frame_id = "map";
  cloud_3d_collisions->header.stamp = ros::Time(0);

  footprint_fifo = new FIFO(512);
}

EnvironmentNav3DCollisionsBase::~EnvironmentNav3DCollisionsBase()
{

	SBPL_PRINTF("destroying XYTHETALATTICE\n");
	if(grid2Dsearchfromstart != NULL)
		delete grid2Dsearchfromstart;
	grid2Dsearchfromstart = NULL;

	if(grid2Dsearchfromgoal != NULL)
		delete grid2Dsearchfromgoal;
	grid2Dsearchfromgoal = NULL;

	if(EnvNAVXYTHETALATCfg.Grid2D != NULL)
	{	
		for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) 
			delete [] EnvNAVXYTHETALATCfg.Grid2D[x];
		delete [] EnvNAVXYTHETALATCfg.Grid2D;
		EnvNAVXYTHETALATCfg.Grid2D = NULL;
	}

	//delete actions
	if(EnvNAVXYTHETALATCfg.ActionsV != NULL)
	{
		for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
			delete [] EnvNAVXYTHETALATCfg.ActionsV[tind];
		delete [] EnvNAVXYTHETALATCfg.ActionsV;
		EnvNAVXYTHETALATCfg.ActionsV = NULL;
	}
	if(EnvNAVXYTHETALATCfg.PredActionsV != NULL)
	{
		delete [] EnvNAVXYTHETALATCfg.PredActionsV;
		EnvNAVXYTHETALATCfg.PredActionsV = NULL;
	}
}

//---------------------------------------------------------------------


//-------------------problem specific and local functions---------------------


static unsigned int inthash(unsigned int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}



void EnvironmentNav3DCollisionsBase::SetConfiguration(int width, int height,
					const unsigned char* mapdata,
					int startx, int starty, int starttheta,
					int goalx, int goaly, int goaltheta,
					double origin_x, double origin_y,
					double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs) {
  EnvNAVXYTHETALATCfg.EnvWidth_c = width;
  EnvNAVXYTHETALATCfg.EnvHeight_c = height;
  EnvNAVXYTHETALATCfg.StartX_c = startx;
  EnvNAVXYTHETALATCfg.StartY_c = starty;
  EnvNAVXYTHETALATCfg.StartTheta = starttheta;
 
  if(EnvNAVXYTHETALATCfg.StartX_c < 0 || EnvNAVXYTHETALATCfg.StartX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETALATCfg.StartY_c < 0 || EnvNAVXYTHETALATCfg.StartY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETALATCfg.StartTheta < 0 || EnvNAVXYTHETALATCfg.StartTheta >= NAVXYTHETALAT_THETADIRS) {
    SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
    throw new SBPL_Exception();
  }
  
  EnvNAVXYTHETALATCfg.EndX_c = goalx;
  EnvNAVXYTHETALATCfg.EndY_c = goaly;
  EnvNAVXYTHETALATCfg.EndTheta = goaltheta;

  if(EnvNAVXYTHETALATCfg.EndX_c < 0 || EnvNAVXYTHETALATCfg.EndX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal goal coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETALATCfg.EndY_c < 0 || EnvNAVXYTHETALATCfg.EndY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal goal coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETALATCfg.EndTheta < 0 || EnvNAVXYTHETALATCfg.EndTheta >= NAVXYTHETALAT_THETADIRS) {
    SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
    throw new SBPL_Exception();
  }

  EnvNAVXYTHETALATCfg.nominalvel_mpersecs = nominalvel_mpersecs;

  EnvNAVXYTHETALATCfg.costmap_origin_x = origin_x;
  EnvNAVXYTHETALATCfg.costmap_origin_y = origin_y;
  EnvNAVXYTHETALATCfg.cellsize_m = cellsize_m;
  EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;


  //allocate the 2D environment
  EnvNAVXYTHETALATCfg.Grid2D = new unsigned char* [EnvNAVXYTHETALATCfg.EnvWidth_c];
  for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
    EnvNAVXYTHETALATCfg.Grid2D[x] = new unsigned char [EnvNAVXYTHETALATCfg.EnvHeight_c];
  }
  
  //environment:
  // always init to 0!
//  if (0 == mapdata) {
    for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
	EnvNAVXYTHETALATCfg.Grid2D[x][y] = 0;
      }
    }
//  }
//  else {
//    for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
//      for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
//			EnvNAVXYTHETALATCfg.Grid2D[x][y] = mapdata[x+y*width];
//      }
//    }
//  }
}

bool EnvironmentNav3DCollisionsBase::ReadinCell(EnvNAVXYTHETALAT3Dcell_t* cell, FILE* fIn)
{
   char sTemp[60];

	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	cell->x = atoi(sTemp);
	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	cell->y = atoi(sTemp);
	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	cell->theta = atoi(sTemp);

    //normalize the angle
	cell->theta = NORMALIZEDISCTHETA(cell->theta, NAVXYTHETALAT_THETADIRS);

	return true;
}

bool EnvironmentNav3DCollisionsBase::ReadinPose(EnvNAVXYTHETALAT3Dpt_t* pose, FILE* fIn)
{
   char sTemp[60];

	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	pose->x = atof(sTemp);
	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	pose->y = atof(sTemp);
	if(fscanf(fIn, "%s", sTemp) == 0)
	   return false;
	pose->theta = atof(sTemp);

	pose->theta = normalizeAngle(pose->theta);

	return true;
}

bool EnvironmentNav3DCollisionsBase::ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn)
{
    char sTemp[1024];
	int dTemp;
    char sExpected[1024];
    int numofIntermPoses;

    //read in actionID
    strcpy(sExpected, "primID:");
    if(fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if(fscanf(fIn, "%d", &pMotPrim->motprimID) != 1)
        return false;

    //read in start angle
    strcpy(sExpected, "startangle_c:");
    if(fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
   if(fscanf(fIn, "%d", &dTemp) == 0)
   {
	   SBPL_ERROR("ERROR reading startangle\n");
       return false;	
   }
   pMotPrim->starttheta_c = dTemp;
 
   //read in end pose
   strcpy(sExpected, "endpose_c:");
   if(fscanf(fIn, "%s", sTemp) == 0)
       return false;
   if(strcmp(sTemp, sExpected) != 0){
       SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
       return false;
   }

   if(ReadinCell(&pMotPrim->endcell, fIn) == false){
		SBPL_ERROR("ERROR: failed to read in endsearchpose\n");
        return false;
   }
   
    //read in action cost
    strcpy(sExpected, "additionalactioncostmult:");
    if(fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if(fscanf(fIn, "%d", &dTemp) != 1)
        return false;
	pMotPrim->additionalactioncostmult = dTemp;
    
    //read in intermediate poses
    strcpy(sExpected, "intermediateposes:");
    if(fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if(fscanf(fIn, "%d", &numofIntermPoses) != 1)
        return false;
	//all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done 
	//after the action is rotated by initial orientation
    for(int i = 0; i < numofIntermPoses; i++){
        EnvNAVXYTHETALAT3Dpt_t intermpose;
        if(ReadinPose(&intermpose, fIn) == false){
            SBPL_ERROR("ERROR: failed to read in intermediate poses\n");
            return false;
        }
		pMotPrim->intermptV.push_back(intermpose);
    }

	//check that the last pose corresponds correctly to the last pose
	EnvNAVXYTHETALAT3Dpt_t sourcepose;
	sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
	sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
	sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, NAVXYTHETALAT_THETADIRS);
	double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x;
	double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y;
	double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta;				
	int endx_c = CONTXY2DISC(mp_endx_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int endy_c = CONTXY2DISC(mp_endy_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int endtheta_c = ContTheta2Disc(mp_endtheta_rad, NAVXYTHETALAT_THETADIRS);
	if(endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y || endtheta_c != pMotPrim->endcell.theta)
	{	
		SBPL_ERROR("ERROR: incorrect primitive %d with startangle=%d last interm point %f %f %f does not match end pose %d %d %d\n", 
			pMotPrim->motprimID, pMotPrim->starttheta_c,
			pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta,
			pMotPrim->endcell.x, pMotPrim->endcell.y,pMotPrim->endcell.theta);	
			return false;
	}

  
    return true;
}



bool EnvironmentNav3DCollisionsBase::ReadMotionPrimitives(FILE* fMotPrims)
{
    char sTemp[1024], sExpected[1024];
    float fTemp;
	int dTemp;
    int totalNumofActions = 0;

    SBPL_PRINTF("Reading in motion primitives...");
    
    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if(fscanf(fMotPrims, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if(fscanf(fMotPrims, "%f", &fTemp) == 0)
        return false;
    if(fabs(fTemp-EnvNAVXYTHETALATCfg.cellsize_m) > ERR_EPS){
        SBPL_ERROR("ERROR: invalid resolution %f (instead of %f) in the dynamics file\n", 
               fTemp, EnvNAVXYTHETALATCfg.cellsize_m);
        return false;
    }

    //read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if(fscanf(fMotPrims, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if(fscanf(fMotPrims, "%d", &dTemp) == 0)
        return false;
    if(dTemp != NAVXYTHETALAT_THETADIRS){
        SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n", 
               dTemp, NAVXYTHETALAT_THETADIRS);
        return false;
    }


    //read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if(fscanf(fMotPrims, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if(fscanf(fMotPrims, "%d", &totalNumofActions) == 0){
        return false;
    }

    for(int i = 0; i < totalNumofActions; i++){
		SBPL_xytheta_mprimitive motprim;

		if(EnvironmentNav3DCollisionsBase::ReadinMotionPrimitive(&motprim, fMotPrims) == false)
			return false;

		EnvNAVXYTHETALATCfg.mprimV.push_back(motprim);

	}
    SBPL_PRINTF("done ");

    return true;
}


void EnvironmentNav3DCollisionsBase::ComputeReplanningDataforAction(EnvNAVXYTHETALATAction_t* action)
{
	int j;

	//iterate over all the cells involved in the action
	EnvNAVXYTHETALAT3Dcell_t startcell3d, endcell3d;
	for(int i = 0; i < (int)action->intersectingcellsV.size(); i++)
	{

		//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
		startcell3d.theta = action->starttheta;
		startcell3d.x = - action->intersectingcellsV.at(i).x;
		startcell3d.y = - action->intersectingcellsV.at(i).y;

		//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
		endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, NAVXYTHETALAT_THETADIRS); 
		endcell3d.x = startcell3d.x + action->dX; 
		endcell3d.y = startcell3d.y + action->dY;

		//store the cells if not already there
		for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
		{
			if(affectedsuccstatesV.at(j) == endcell3d)
				break;
		}
		if (j == (int)affectedsuccstatesV.size())
			affectedsuccstatesV.push_back(endcell3d);

		for(j = 0; j < (int)affectedpredstatesV.size(); j++)
		{
			if(affectedpredstatesV.at(j) == startcell3d)
				break;
		}
		if (j == (int)affectedpredstatesV.size())
			affectedpredstatesV.push_back(startcell3d);

    }//over intersecting cells

	

	//add the centers since with h2d we are using these in cost computations
	//---intersecting cell = origin
	//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
	startcell3d.theta = action->starttheta;
	startcell3d.x = - 0;
	startcell3d.y = - 0;

	//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
	endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, NAVXYTHETALAT_THETADIRS); 
	endcell3d.x = startcell3d.x + action->dX; 
	endcell3d.y = startcell3d.y + action->dY;

	//store the cells if not already there
	for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
	{
		if(affectedsuccstatesV.at(j) == endcell3d)
			break;
	}
	if (j == (int)affectedsuccstatesV.size())
		affectedsuccstatesV.push_back(endcell3d);

	for(j = 0; j < (int)affectedpredstatesV.size(); j++)
	{
		if(affectedpredstatesV.at(j) == startcell3d)
			break;
	}
	if (j == (int)affectedpredstatesV.size())
		affectedpredstatesV.push_back(startcell3d);


	//---intersecting cell = outcome state
	//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
	startcell3d.theta = action->starttheta;
	startcell3d.x = - action->dX;
	startcell3d.y = - action->dY;

	//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
	endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, NAVXYTHETALAT_THETADIRS); 
	endcell3d.x = startcell3d.x + action->dX; 
	endcell3d.y = startcell3d.y + action->dY;

	for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
	{
		if(affectedsuccstatesV.at(j) == endcell3d)
			break;
	}
	if (j == (int)affectedsuccstatesV.size())
		affectedsuccstatesV.push_back(endcell3d);

	for(j = 0; j < (int)affectedpredstatesV.size(); j++)
	{
		if(affectedpredstatesV.at(j) == startcell3d)
			break;
	}
	if (j == (int)affectedpredstatesV.size())
		affectedpredstatesV.push_back(startcell3d);


}


//computes all the 3D states whose outgoing actions are potentially affected when cell (0,0) changes its status
//it also does the same for the 3D states whose incoming actions are potentially affected when cell (0,0) changes its status
void EnvironmentNav3DCollisionsBase::ComputeReplanningData()
{

    //iterate over all actions
	//orientations
	for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
    {        
        //actions
		for(int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
		{
            //compute replanning data for this action 
			ComputeReplanningDataforAction(&EnvNAVXYTHETALATCfg.ActionsV[tind][aind]);
		}
	}
}

//here motionprimitivevector contains actions only for 0 angle
void EnvironmentNav3DCollisionsBase::PrecomputeActionswithBaseMotionPrimitive(vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{

	SBPL_PRINTF("Pre-computing action data using base motion primitives...\n");
	EnvNAVXYTHETALATCfg.ActionsV = new EnvNAVXYTHETALATAction_t* [NAVXYTHETALAT_THETADIRS];
	EnvNAVXYTHETALATCfg.PredActionsV = new vector<EnvNAVXYTHETALATAction_t*> [NAVXYTHETALAT_THETADIRS];

	//iterate over source angles
	for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, NAVXYTHETALAT_THETADIRS);
		EnvNAVXYTHETALATCfg.ActionsV[tind] = new EnvNAVXYTHETALATAction_t[motionprimitiveV->size()];

		//compute sourcepose
		EnvNAVXYTHETALAT3Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETALAT_THETADIRS);

		//iterate over motion primitives
		for(size_t aind = 0; aind < motionprimitiveV->size(); aind++)
		{
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
			double mp_endx_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].x;
			double mp_endy_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].y;
			double mp_endtheta_rad = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].theta;
			
			double endx = sourcepose.x + (mp_endx_m*cos(sourcepose.theta) - mp_endy_m*sin(sourcepose.theta));
			double endy = sourcepose.y + (mp_endx_m*sin(sourcepose.theta) + mp_endy_m*cos(sourcepose.theta));
			
			int endx_c = CONTXY2DISC(endx, EnvNAVXYTHETALATCfg.cellsize_m);
			int endy_c = CONTXY2DISC(endy, EnvNAVXYTHETALATCfg.cellsize_m);

			
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = ContTheta2Disc(mp_endtheta_rad+sourcepose.theta, NAVXYTHETALAT_THETADIRS);
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = endx_c;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = endy_c;
			if(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY != 0 || EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX != 0)
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM*EnvNAVXYTHETALATCfg.cellsize_m/EnvNAVXYTHETALATCfg.nominalvel_mpersecs*
								sqrt((double)(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX + 
								EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY))));
			else //cost of turn in place
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETALAT_COSTMULT_MTOMM*
						EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs*fabs(computeMinUnsignedAngleDiff(mp_endtheta_rad,0))/(PI_CONST/4.0));

			//compute and store interm points as well as intersecting cells
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.clear();
			EnvNAVXYTHETALAT3Dcell_t previnterm3Dcell;
			previnterm3Dcell.theta = previnterm3Dcell.x = previnterm3Dcell.y = 0;
			for (int pind = 0; pind < (int)motionprimitiveV->at(aind).intermptV.size(); pind++)
			{
				EnvNAVXYTHETALAT3Dpt_t intermpt = motionprimitiveV->at(aind).intermptV[pind];
		
				//rotate it appropriately
				double rotx = intermpt.x*cos(sourcepose.theta) - intermpt.y*sin(sourcepose.theta);
				double roty = intermpt.x*sin(sourcepose.theta) + intermpt.y*cos(sourcepose.theta);
				intermpt.x = rotx;
				intermpt.y = roty;
				intermpt.theta = normalizeAngle(sourcepose.theta + intermpt.theta);

				//store it (they are with reference to 0,0,stattheta (not sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

				//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y
				EnvNAVXYTHETALAT3Dpt_t pose;
				pose = intermpt;
				pose.x += sourcepose.x;
				pose.y += sourcepose.y;
				CalculateFootprintForPose(pose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

				//now also store the intermediate discretized cell if not there already
				EnvNAVXYTHETALAT3Dcell_t interm3Dcell;
				interm3Dcell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETALATCfg.cellsize_m);
				interm3Dcell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETALATCfg.cellsize_m);
				interm3Dcell.theta = ContTheta2Disc(pose.theta, NAVXYTHETALAT_THETADIRS); 
				if(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.size() == 0 || 
					previnterm3Dcell.theta != interm3Dcell.theta || previnterm3Dcell.x != interm3Dcell.x || previnterm3Dcell.y != interm3Dcell.y)
				{
					EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.push_back(interm3Dcell);
				}
				previnterm3Dcell = interm3Dcell;

			}

			//now remove the source footprint
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_FPRINTF(fDeb, "action tind=%d aind=%d: dX=%d dY=%d endtheta=%d (%.2f degs -> %.2f degs) cost=%d (mprim: %.2f %.2f %.2f)\n",
				tind, aind, 			
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, sourcepose.theta*180/PI_CONST, 
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size()-1].theta*180/PI_CONST,	
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost,
				mp_endx_m, mp_endy_m, mp_endtheta_rad);
#endif

			//add to the list of backward actions
			int targettheta = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETALAT_THETADIRS;
			 EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));

		}
	}

	//set number of actions
	EnvNAVXYTHETALATCfg.actionwidth = motionprimitiveV->size();


	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data based on motion primitives\n");


}


//here motionprimitivevector contains actions for all angles
void EnvironmentNav3DCollisionsBase::PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
  clock_t t0 = clock();
	ROS_INFO("Pre-computing action data using motion primitives for every angle...");
	EnvNAVXYTHETALATCfg.ActionsV = new EnvNAVXYTHETALATAction_t* [NAVXYTHETALAT_THETADIRS];
	EnvNAVXYTHETALATCfg.PredActionsV = new vector<EnvNAVXYTHETALATAction_t*> [NAVXYTHETALAT_THETADIRS];

	if(motionprimitiveV->size()%NAVXYTHETALAT_THETADIRS != 0)
	{
		SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
		throw new SBPL_Exception();
	}

	EnvNAVXYTHETALATCfg.actionwidth = ((int)motionprimitiveV->size())/NAVXYTHETALAT_THETADIRS;

	//iterate over source angles
	int maxnumofactions = 0;
	for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
	{
    clock_t t2 = clock();
		SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, NAVXYTHETALAT_THETADIRS);

		EnvNAVXYTHETALATCfg.ActionsV[tind] = new EnvNAVXYTHETALATAction_t[EnvNAVXYTHETALATCfg.actionwidth];  

		//compute sourcepose
		EnvNAVXYTHETALAT3Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETALAT_THETADIRS);


		//iterate over motion primitives
		int numofactions = 0;
		int aind = -1;
		for(int mind = 0; mind < (int)motionprimitiveV->size(); mind++)
		{
      clock_t t3 = clock();
			//find a motion primitive for this angle
			if(motionprimitiveV->at(mind).starttheta_c != tind)
				continue;
			
			aind++;
			numofactions++;
			
			//action index
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;

			//start angle
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;

			//compute dislocation
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;

			//compute cost
			if(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY != 0 || EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX != 0)
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM*EnvNAVXYTHETALATCfg.cellsize_m/EnvNAVXYTHETALATCfg.nominalvel_mpersecs*
								sqrt((double)(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX + 
								EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY))));
			else //cost of turn in place
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETALAT_COSTMULT_MTOMM*
						EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs*
						fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETALAT_THETADIRS),
														DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta, NAVXYTHETALAT_THETADIRS)))/(PI_CONST/4.0));
			//use any additional cost multiplier
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;

			//compute and store interm points as well as intersecting cells
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.clear();
			EnvNAVXYTHETALAT3Dcell_t previnterm3Dcell;
			previnterm3Dcell.theta = 0; previnterm3Dcell.x = 0; previnterm3Dcell.y = 0;			
      set<pair<int,int> > cell_set;
      clock_t foot_time = 0;
      clock_t t4 = clock();
			for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++)
			{
				EnvNAVXYTHETALAT3Dpt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
		
				//store it (they are with reference to 0,0,stattheta (not sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

				//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y
				EnvNAVXYTHETALAT3Dpt_t pose;
				pose = intermpt;
				pose.x += sourcepose.x;
				pose.y += sourcepose.y;
        clock_t t6 = clock();
				//CalculateFootprintForPose(pose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);
        printFP = false;//tind==0 && aind==0;
        //printf("before cell_set size %d\n",cell_set.size());
				CalculateFootprintForPose(pose, &cell_set);
        //printf("after cell_set size %d\n",cell_set.size());
        printFP = false;
        clock_t t7 = clock();
        //printf("calc footprint took %f\n", ((double)(t7-t6))/CLOCKS_PER_SEC);
        foot_time += t7-t6;
			
				//now also store the intermediate discretized cell if not there already
				EnvNAVXYTHETALAT3Dcell_t interm3Dcell;
				interm3Dcell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETALATCfg.cellsize_m);
				interm3Dcell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETALATCfg.cellsize_m);
				interm3Dcell.theta = ContTheta2Disc(pose.theta, NAVXYTHETALAT_THETADIRS); 
				if(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.size() == 0 || 
					previnterm3Dcell.theta != interm3Dcell.theta || previnterm3Dcell.x != interm3Dcell.x || previnterm3Dcell.y != interm3Dcell.y)
				{
					EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.push_back(interm3Dcell);
				}
				previnterm3Dcell = interm3Dcell;
			}

      EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.reserve(cell_set.size());
      for(set<pair<int,int> >::iterator it = cell_set.begin(); it!=cell_set.end(); it++){
        sbpl_2Dcell_t cell;
        cell.x = it->first;
        cell.y = it->second;
        EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.push_back(cell);
      }
      //printf("num cells %d\n",EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.size());

      //printf("foot time took %f\n", ((double)(foot_time))/CLOCKS_PER_SEC);
      //printf("interm points took %f\n", ((double)(clock()-t4))/CLOCKS_PER_SEC);

			//now remove the source footprint
      clock_t t5 = clock();

      //printf("before remove source %d\n",EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.size());
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);
      //printf("after remove source %d\n",EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.size());
      //printf("remove footprint took %f\n", ((double)(clock()-t5))/CLOCKS_PER_SEC);

#if DEBUG
			SBPL_FPRINTF(fDeb, "action tind=%d aind=%d: dX=%d dY=%d endtheta=%d (%.2f degs -> %.2f degs) cost=%d (mprimID %d: %d %d %d) numofintermcells = %d numofintercells=%d\n",
				tind, aind, 			
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, 
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[0].theta*180/PI_CONST, 
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size()-1].theta*180/PI_CONST,	
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost,
				motionprimitiveV->at(mind).motprimID, 
				motionprimitiveV->at(mind).endcell.x, motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.theta,
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.size(),
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.size()); 
#endif

			//add to the list of backward actions
			int targettheta = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETALAT_THETADIRS;
			 EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));

      //printf("motion %d took %f\n",mind, ((double)(clock()-t3))/CLOCKS_PER_SEC);
		}

		if(maxnumofactions < numofactions)
			maxnumofactions = numofactions;
    printf("theta %d took %f\n",tind, ((double)(clock()-t2))/CLOCKS_PER_SEC);
	}



	//at this point we don't allow nonuniform number of actions
	if(motionprimitiveV->size() != (size_t)(NAVXYTHETALAT_THETADIRS*maxnumofactions))
	{
		SBPL_ERROR("ERROR: nonuniform number of actions is not supported (maxnumofactions=%d while motprims=%d thetas=%d\n",
				maxnumofactions, (unsigned int)motionprimitiveV->size(), NAVXYTHETALAT_THETADIRS);
		throw new SBPL_Exception();
	}

	//now compute replanning data
  clock_t t1 = clock();
	ComputeReplanningData();
  printf("replanning data %f\n",((double)(clock()-t1))/CLOCKS_PER_SEC);

	SBPL_PRINTF("done pre-computing action data based on motion primitives\n");


  printf("total pre-computing actions %f\n",((double)(clock()-t0))/CLOCKS_PER_SEC);
}

void EnvironmentNav3DCollisionsBase::PrecomputeActions()
{

	//construct list of actions
	ROS_WARN("Pre-computing action data (NOT using motion primitives)...");
	EnvNAVXYTHETALATCfg.ActionsV = new EnvNAVXYTHETALATAction_t* [NAVXYTHETALAT_THETADIRS];
	EnvNAVXYTHETALATCfg.PredActionsV = new vector<EnvNAVXYTHETALATAction_t*> [NAVXYTHETALAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;
	//iterate over source angles
	for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("processing angle %d\n", tind);
		EnvNAVXYTHETALATCfg.ActionsV[tind] = new EnvNAVXYTHETALATAction_t[EnvNAVXYTHETALATCfg.actionwidth];

		//compute sourcepose
		EnvNAVXYTHETALAT3Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETALAT_THETADIRS);

		//the construction assumes that the robot first turns and then goes along this new theta
		int aind = 0;
		for(; aind < 3; aind++)
		{
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = (tind + aind - 1)%NAVXYTHETALAT_THETADIRS; //-1,0,1
			double angle = DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETALAT_THETADIRS);
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = (int)(cos(angle) + 0.5*(cos(angle)>0?1:-1));
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = (int)(sin(angle) + 0.5*(sin(angle)>0?1:-1));
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM*EnvNAVXYTHETALATCfg.cellsize_m/EnvNAVXYTHETALATCfg.nominalvel_mpersecs*sqrt((double)(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX + 
					EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY))));

			//compute intersecting cells
			EnvNAVXYTHETALAT3Dpt_t pose;
			pose.x = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.cellsize_m);
			pose.y = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETALATCfg.cellsize_m);
			pose.theta = angle;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			CalculateFootprintForPose(pose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
				tind, aind, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, angle, 
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

			//add to the list of backward actions
			int targettheta = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETALAT_THETADIRS;
			 EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));

		}

		//decrease and increase angle without movement
		aind = 3;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = tind-1;
		if(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta < 0) EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta += NAVXYTHETALAT_THETADIRS;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = 0;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = 0;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		EnvNAVXYTHETALAT3Dpt_t pose;
		pose.x = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETALATCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETALAT_THETADIRS);
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
			tind, aind, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETALAT_THETADIRS),
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		int targettheta = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + NAVXYTHETALAT_THETADIRS;
		 EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));


		aind = 4;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = (tind + 1)%NAVXYTHETALAT_THETADIRS; 
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = 0;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = 0;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		pose.x = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETALATCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETALAT_THETADIRS);
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);


#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
			tind, aind, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETALAT_THETADIRS),
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		targettheta = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + NAVXYTHETALAT_THETADIRS;
		 EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));

	}

	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data\n");


}



void EnvironmentNav3DCollisionsBase::InitializeEnvConfig(vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
	//aditional to configuration file initialization of EnvNAVXYTHETALATCfg if necessary

	//dXY dirs
	EnvNAVXYTHETALATCfg.dXY[0][0] = -1;
	EnvNAVXYTHETALATCfg.dXY[0][1] = -1;
	EnvNAVXYTHETALATCfg.dXY[1][0] = -1;
	EnvNAVXYTHETALATCfg.dXY[1][1] = 0;
	EnvNAVXYTHETALATCfg.dXY[2][0] = -1;
	EnvNAVXYTHETALATCfg.dXY[2][1] = 1;
	EnvNAVXYTHETALATCfg.dXY[3][0] = 0;
	EnvNAVXYTHETALATCfg.dXY[3][1] = -1;
	EnvNAVXYTHETALATCfg.dXY[4][0] = 0;
	EnvNAVXYTHETALATCfg.dXY[4][1] = 1;
	EnvNAVXYTHETALATCfg.dXY[5][0] = 1;
	EnvNAVXYTHETALATCfg.dXY[5][1] = -1;
	EnvNAVXYTHETALATCfg.dXY[6][0] = 1;
	EnvNAVXYTHETALATCfg.dXY[6][1] = 0;
	EnvNAVXYTHETALATCfg.dXY[7][0] = 1;
	EnvNAVXYTHETALATCfg.dXY[7][1] = 1;


	EnvNAVXYTHETALAT3Dpt_t temppose;
	temppose.x = 0.0;
	temppose.y = 0.0;
	temppose.theta = 0.0;
	vector<sbpl_2Dcell_t> footprint;
	CalculateFootprintForPose(temppose, &footprint);
	SBPL_PRINTF("number of cells in footprint of the robot = %d\n", (unsigned int)footprint.size());

#if DEBUG
	SBPL_FPRINTF(fDeb, "footprint cells (size=%d):\n", footprint.size());
	for(int i = 0; i < (int) footprint.size(); i++)
	{
		SBPL_FPRINTF(fDeb, "%d %d (cont: %.3f %.3f)\n", footprint.at(i).x, footprint.at(i).y, 
			DISCXY2CONT(footprint.at(i).x, EnvNAVXYTHETALATCfg.cellsize_m), 
			DISCXY2CONT(footprint.at(i).y, EnvNAVXYTHETALATCfg.cellsize_m));
	}
#endif


	if(motionprimitiveV == NULL)
		PrecomputeActions();
	else
		PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);


}

bool EnvironmentNav3DCollisionsBase::IsValidConfiguration(int X, int Y, int Theta)
{
	vector<sbpl_2Dcell_t> footprint;
	EnvNAVXYTHETALAT3Dpt_t pose;

	//compute continuous pose
	pose.x = DISCXY2CONT(X, EnvNAVXYTHETALATCfg.cellsize_m);
	pose.y = DISCXY2CONT(Y, EnvNAVXYTHETALATCfg.cellsize_m);
	pose.theta = DiscTheta2Cont(Theta, NAVXYTHETALAT_THETADIRS);

	//compute footprint cells
	CalculateFootprintForPose(pose, &footprint);

	// TODO: use regular collision check instead
	// (this fct. only used by SetStart and SetGoal)
	//iterate over all footprint cells
	for(int find = 0; find < (int)footprint.size(); find++)
	{
		int x = footprint.at(find).x;
		int y = footprint.at(find).y;

		if (!IsValidCell(x, y))
		{
			ROS_DEBUG("Not valid at %d %d", x, y);
			return false;
		}
	}

	return true;
}

bool EnvironmentNav3DCollisionsBase::updateKinematicState(const motion_planning_msgs::RobotState &robot_state, vector<sbpl_2Dpt_t>* new_fp){
	if (!planning_environment::setRobotStateAndComputeTransforms(robot_state, *m_kinematicState))
	{
		ROS_WARN("Robot State to kinematic model update incomplete");
	} else{
		ROS_INFO("Kinematic model updated from robot state");
	}

	// recalculate footprint for 2D checks

	bool updated = updateFootprint();

	// reset counters
	m_num3DCollChecks = 0;
	m_num2DCollChecks = 0;

  for(unsigned int i=0; i<downprojectedFootprint.size(); i++)
    new_fp->push_back(downprojectedFootprint[i]);
  return updated;
}

void EnvironmentNav3DCollisionsBase::updateRobotPosition(double x, double y, double theta){
	btTransform cur(tf::createQuaternionFromYaw(theta), btVector3(x + EnvNAVXYTHETALATCfg.costmap_origin_x, y + EnvNAVXYTHETALATCfg.costmap_origin_x, 0.0));
	m_kinematicState->getJointStateVector()[0]->setJointStateValues(cur);
	m_kinematicState->updateKinematicLinks();
}

void EnvironmentNav3DCollisionsBase::updateAttachedObjects(const mapping_msgs::AttachedCollisionObject& coll){
	// kinematic state needs to be freed because of mutex lock
	if (m_kinematicState){
		delete m_kinematicState;
		m_kinematicState = NULL;
	}
	if(!m_planningCollisionModel.addAttachedObject(coll)){
    ROS_ERROR("Failed to add attached object to collision model");
    return;
  }
	m_kinematicState = new planning_models::KinematicState(m_planningCollisionModel.getKinematicModel());
  //printf("msg type %d, shape type %d\n",coll.object.shapes.back().type, m_kinematicState->getAttachedBodyState(coll.object.id)->getAttachedBodyModel()->getAttachedLinkModel()->getLinkShape()->type);

  if(coll.object.operation.operation==mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT){
    basketRadius = coll.object.shapes.back().dimensions.front();
    bool haveObject = false;
    for(unsigned int i = 0; i < m_footprintLinkNames.size(); i++){
      if(m_footprintLinkNames[i].compare(coll.object.id)==0){
        haveObject = true;
        break;
      }
    }
    if(!haveObject)
      m_footprintLinkNames.push_back(coll.object.id);
  }
  else if(coll.object.operation.operation==mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT){
    for(unsigned int i = 0; i < m_footprintLinkNames.size(); i++){
      if(m_footprintLinkNames[i].compare(coll.object.id)==0){
        m_footprintLinkNames.erase(m_footprintLinkNames.begin()+i);
        break;
      }
    }
  }
}

bool EnvironmentNav3DCollisionsBase::updateFootprint(){
	// TODO: add attached objects?

	m_kinematicState->getJointStateVector()[0]->setJointStateValues(tf::Transform(tf::createIdentityQuaternion()));
	m_kinematicState->updateKinematicLinks();

	std::vector<cv::Point> points;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  //cloud_projected->header.frame_id = "base_footprint";
  //cloud_projected->header.stamp = ros::Time(0);
	for(unsigned int i = 0; i < m_footprintLinkNames.size(); i++) {
    printf("projecting link %s\n",m_footprintLinkNames[i].c_str());
    if(m_footprintLinkNames[i].compare("basket")!=0){
      const planning_models::KinematicState::LinkState* ls = m_kinematicState->getLinkState(m_footprintLinkNames[i]);
      const shapes::Shape* shape;
      if(ls==NULL){
        /*
        //check if this is an attached object
        planning_models::KinematicState::AttachedBodyState* abs = m_kinematicState->getAttachedBodyState(m_footprintLinkNames[i]);
        if(abs==NULL){
        */
          ROS_WARN("Link (%s) does not exist in the kinematic state!",m_footprintLinkNames[i].c_str());
          continue;
        /*
        }
        const planning_models::KinematicModel::AttachedBodyModel* abm = abs->getAttachedBodyModel();
        if(abm==NULL){
          ROS_WARN("Attached object %s doesn't have a body model!",m_footprintLinkNames[i].c_str());
          continue;
        }
        //printf("2\n");
        shape = abm->getAttachedLinkModel()->getLinkShape();
        //printf("3\n");
        */
      }
      else{
        if(ls->getLinkModel()->getLinkShape() == NULL){
          ROS_WARN("No Shape for link %s", m_footprintLinkNames[i].c_str());
          continue;
        }
        shape = ls->getLinkModel()->getLinkShape();
      }

      printf("shape ptr %x\n",shape);
      printf("type %d\n",shape->type);
      if(shape->type == shapes::MESH){
        const shapes::Mesh *mesh = dynamic_cast<const shapes::Mesh*>(shape);
        if(mesh == NULL){
          ROS_WARN("Could not get mesh for link %s", m_footprintLinkNames[i].c_str());
          continue;
        }
        if (mesh->vertexCount > 0 && mesh->triangleCount > 0)
        {
          const btTransform& trans = ls->getGlobalCollisionBodyTransform();

          for (unsigned int i = 0 ; i < mesh->vertexCount ; ++i)
          {
            unsigned int i3 = i * 3;
            tf::Vector3 point3d(mesh->vertices[i3], mesh->vertices[i3 + 1], mesh->vertices[i3 + 2]);
            point3d = trans * point3d;
            cv::Point pt;
            //pcl::PointXYZ pt;
            pt.x = floor(point3d.x() / EnvNAVXYTHETALATCfg.cellsize_m);
            pt.y = floor(point3d.y() / EnvNAVXYTHETALATCfg.cellsize_m);
            //pt.x = point3d.x(); //floor(point3d.x() / EnvNAVXYTHETALATCfg.cellsize_m);
            //pt.y = point3d.y(); //floor(point3d.y() / EnvNAVXYTHETALATCfg.cellsize_m);
            //pt.z = 0;
            //cloud_projected->points.push_back(pt);
            points.push_back(pt);
          }

        }
      }
    }
    else{
      printf("4\n");
      //const shapes::Shape* shape = m_kinematicState->getAttachedBodyState(m_footprintLinkNames[i])->getAttachedBodyModel()->getAttachedLinkModel()->getLinkShape();
      //const shapes::Cylinder *c = dynamic_cast<const shapes::Cylinder*>(shape);
      const btTransform& trans = m_kinematicState->getAttachedBodyState(m_footprintLinkNames[i])->getGlobalCollisionBodyTransforms().front();
      printf("5\n");
      double r = basketRadius;//c->radius;
      int numpts = 100;
      for(unsigned int i=0; i<numpts; i++){
        double a = 2*M_PI*double(i)/numpts;
        tf::Vector3 point3d(r*cos(a),r*sin(a),0);
        point3d = trans * point3d;
        cv::Point pt;
        pt.x = floor(point3d.x() / EnvNAVXYTHETALATCfg.cellsize_m);
        pt.y = floor(point3d.y() / EnvNAVXYTHETALATCfg.cellsize_m);
        points.push_back(pt);
      }
      printf("6\n");
    }
	}
  printf("done with link projection\n");
  //footPointsPub.publish(cloud_projected);

	if (points.size() < 3){
	//if (cloud_projected->size() < 3){
		ROS_ERROR("Number of points from link meshes too small to compute footprint");
		return false;
	}

	vector<int> hull;
	cv::convexHull(cv::Mat(points), hull);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr hull (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::ConcaveHull<pcl::PointXYZ> chull;
  //chull.setInputCloud (cloud_projected);
  //chull.setAlpha(1.0);
  //chull.reconstruct (*hull);

	ROS_INFO("Convex hull of footprint computed, %d points", int(hull.size()));
	//ROS_INFO("Convex hull of footprint computed, %d points", int(hull->points.size()));
	geometry_msgs::PolygonStamped footprint_poly;
	footprint_poly.header.frame_id = "base_footprint";
	footprint_poly.header.stamp = ros::Time::now();

	footprint_poly.polygon.points.resize(hull.size());
	//footprint_poly.polygon.points.resize(hull->points.size());
  bool footprintChanged = false;
  if(hull.size() != downprojectedFootprint.size()){
  //if(hull->points.size() != downprojectedFootprint.size()){
    footprintChanged = true;
    printf("footprint size changed %d->%d\n",downprojectedFootprint.size(),hull.size());
    //printf("footprint size changed %d->%d\n",downprojectedFootprint.size(),hull->points.size());
  }
  downprojectedFootprint.resize(hull.size());
  //downprojectedFootprint.resize(hull->points.size());

    // compute center to enlarge with padding:
    tf::Vector3 center(0,0,0);
    for(unsigned int i = 0; i < hull.size(); ++i){
    //for(unsigned int i = 0; i < hull->points.size(); ++i){
    	center.setX(center.x() + footprint_poly.polygon.points[i].x);
    	center.setY(center.y() + footprint_poly.polygon.points[i].y);
    }
    center / double(hull.size());
    //center / double(hull->points.size());
    double footprintPadding = 0.02; // 2cm => param?

	for(unsigned int i = 0; i < hull.size(); ++i){
	//for(unsigned int i = 0; i < hull->points.size(); ++i){
    tf::Vector3 p((double(points[hull[i]].x)+0.5) * EnvNAVXYTHETALATCfg.cellsize_m,
        (double(points[hull[i]].y)+0.5) * EnvNAVXYTHETALATCfg.cellsize_m, 0);
    //tf::Vector3 p(hull->points[i].x, hull->points[i].y, 0);
    p += (p-center).normalize() * footprintPadding;

    footprint_poly.polygon.points[i].x = p.x();
    footprint_poly.polygon.points[i].y = p.y();
    footprint_poly.polygon.points[i].z = 0.0;

    if(!footprintChanged){
      double dx = downprojectedFootprint[i].x-p.x();
      double dy = downprojectedFootprint[i].y-p.y();
      if(sqrt(dx*dx+dy*dy) > EnvNAVXYTHETALATCfg.cellsize_m/2){
        footprintChanged = true;
        printf("footprint point %d changed (%f,%f)->(%f,%f)\n",i,downprojectedFootprint[i].x,downprojectedFootprint[i].y,p.x(),p.y());
      }
    }
    
    downprojectedFootprint[i].x = p.x();
    downprojectedFootprint[i].y = p.y();
	}

	//m_footprintPolygonPub.publish(footprint_poly);

	// recompute footprints along actions (rollout)
  /*
  if(footprintChanged)
    PrecomputeActionswithCompleteMotionPrimitive(&EnvNAVXYTHETALATCfg.mprimV);
  */
  return footprintChanged;
}

void EnvironmentNav3DCollisionsBase::updateCollisionObjects(const mapping_msgs::CollisionObject& coll){

	if (coll.poses.size() == 0){
		ROS_INFO("Skipping empty collision map");
	} else {
		ROS_INFO("Updating collision map");
		m_planningCollisionModel.addStaticObject(coll);

	//	std::vector<shapes::Shape*> shapes;
	//	std::vector<btTransform> poses;
	//	for (unsigned i = 0; i < coll.poses.size(); ++i){
	//		double size = coll.shapes[i].dimensions[0];
	//		shapes.push_back(new shapes::Box(size,size,size));
	//		tf::Pose pose;
	//		tf::poseMsgToTF(coll.poses[i], pose);
	//		poses.push_back(pose);
	//	}
	//
	//	m_planningCollisionModel.addStaticObject("map", shapes, poses, 0.0); //TODO: param for padding?


		// ignore collisions by the wheels:
		collision_space::EnvironmentModel::AllowedCollisionMatrix acm = m_planningCollisionModel.getCurrentAllowedCollisionMatrix();
		acm.changeEntry("fl_caster_l_wheel_link", true);
		acm.changeEntry("fl_caster_r_wheel_link", true);
		acm.changeEntry("fl_caster_rotation_link", true);
		acm.changeEntry("bl_caster_l_wheel_link", true);
		acm.changeEntry("bl_caster_r_wheel_link", true);
		acm.changeEntry("bl_caster_rotation_link", true);
		acm.changeEntry("fr_caster_l_wheel_link", true);
		acm.changeEntry("fr_caster_r_wheel_link", true);
		acm.changeEntry("fr_caster_rotation_link", true);
		acm.changeEntry("br_caster_l_wheel_link", true);
		acm.changeEntry("br_caster_r_wheel_link", true);
		acm.changeEntry("br_caster_rotation_link", true);

    if(use_multi_layer){
      ROS_INFO("Using multiple layers so we can disable most 3D collision checking....we only need the arms");
      acm.changeEntry("base_link", true);
      acm.changeEntry("torso_lift_link", true);
      acm.changeEntry("head_pan_link", true);
      acm.changeEntry("head_tilt_link", true);
      acm.changeEntry("sensor_mount_link", true);
      acm.changeEntry("double_stereo_link", true);
      acm.changeEntry("narrow_stereo_link", true);
      acm.changeEntry("wide_stereo_link", true);
      acm.changeEntry("imu_link", true);
      acm.changeEntry("l_torso_lift_side_plate_link", true);
      acm.changeEntry("laser_tilt_mount_link", true);
      acm.changeEntry("laser_tilt_link", true);
      acm.changeEntry("r_torso_lift_side_plate_link", true);
      acm.changeEntry("base_bellow_link", true);
      acm.changeEntry("base_laser_link", true);
    }

		m_planningCollisionModel.setAlteredAllowedCollisionMatrix(acm);
    if(use_multi_layer)
      m_planningCollisionModel.disableCollisionsForNonUpdatedLinks("arms");
	}

	m_collisionReceived = true;
	m_num3DCollChecks = 0;
	m_num2DCollChecks = 0;

}

/// 3D collision check at x,y,theta in costmap coordinates
bool EnvironmentNav3DCollisionsBase::isIn3DCollision(double x, double y, double theta){
	ROS_DEBUG("3D collision check at %f %f %f", x, y, theta);
	if (!m_collisionReceived){
		ROS_ERROR("pose_follower_3d did not receive any 3D CollisionObject, no 3D collision check possible");
		return true;
	}

	//m_num3DCollChecks++;

	// move robot to planned base configuration:
	updateRobotPosition(x,y,theta);
	// this is the collision check:
	return m_planningCollisionModel.isKinematicStateInEnvironmentCollision(*m_kinematicState);
}

void EnvironmentNav3DCollisionsBase::visualize3DCollsisions(){
	visualization_msgs::MarkerArray arr;
	std_msgs::ColorRGBA col;
	col.r = 1.0;
	col.g = 0.0;
	col.b = 0.0;
	col.a = 0.9;
	m_planningCollisionModel.getAllCollisionPointMarkers(*m_kinematicState, arr, col, ros::Duration(2.0));
	col.g = 1.0;
	m_planningCollisionModel.getAttachedCollisionObjectMarkers(*m_kinematicState, arr, "attached", col, ros::Duration(2.0));
	//planningCollisionModel.getStaticCollisionObjectMarkers(m_kinematicState, arr, col, ros::Duration(0.1));
	m_collisionMarkerPub.publish(arr);
}


int EnvironmentNav3DCollisionsBase::GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action, bool* possible_collision)
{
	if(!IsWithinMapCell(SourceX, SourceY)){
    //printf("base sink off map\n");
		return INFINITECOST;
  }
	if(!IsWithinMapCell(SourceX + action->dX, SourceY + action->dY)){
    //printf("base sink off map\n");
		return INFINITECOST;
  }

	//need to iterate over discretized center cells and compute cost based on them
	unsigned char maxcellcost = 0;
	EnvNAVXYTHETALAT3Dcell_t interm3Dcell;
	for(int i = 0; i < (int)action->interm3DcellsV.size(); i++)
	{

		interm3Dcell = action->interm3DcellsV.at(i);
		interm3Dcell.x = interm3Dcell.x + SourceX;
		interm3Dcell.y = interm3Dcell.y + SourceY;
		ROS_DEBUG("Checking intermediate center cell at %d %d", interm3Dcell.x, interm3Dcell.y);
		
		if(!IsWithinMapCell(interm3Dcell.x, interm3Dcell.y)){
      //printf("base center off map\n");
			return INFINITECOST;
    }

		maxcellcost = __max(maxcellcost, EnvNAVXYTHETALATCfg.Grid2D[interm3Dcell.x][interm3Dcell.y]);

		if(maxcellcost >= EnvNAVXYTHETALATCfg.cost_inscribed_thresh){
      //printf("base inscribed collision\n");
      return INFINITECOST;
    }
	}
  //printf("base max cell cost %d\n",maxcellcost);

  //TODO: we also don't need to go in here if possible_collision is already true and we are far from map boundaries
  if(maxcellcost >= EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh){

    //check collisions that for the particular footprint orientation along the action
    sbpl_2Dcell_t cell;
    if(EnvNAVXYTHETALATCfg.FootprintPolygon.size() > 1)
    {
      ROS_DEBUG("Checking footprint polygon at %d %d", SourceX, SourceY);
      //m_num2DCollChecks++;

      for(int i = 0; i < (int)action->intersectingcellsV.size(); i++)
      {
        //get the cell in the map
        cell = action->intersectingcellsV.at(i);
        cell.x = cell.x + SourceX;
        cell.y = cell.y + SourceY;
        
        //check validity
        if(!IsWithinMapCell(cell.x, cell.y)){
          //printf("base footprint off map\n");
          return INFINITECOST;
        }

        if(EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y] >= EnvNAVXYTHETALATCfg.obsthresh){
          //printf("base footprint collision\n");
          return INFINITECOST;
        }
      }
    }
  }


	//to ensure consistency of h2D:
	maxcellcost = __max(maxcellcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX][SourceY]);
	int currentmaxcost = (int)__max(maxcellcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);

	//use cell cost as multiplicative factor
	return action->cost*(currentmaxcost+1);
}



double EnvironmentNav3DCollisionsBase::EuclideanDistance_m(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    return EnvNAVXYTHETALATCfg.cellsize_m*sqrt((double)sqdist);

}

//calculates a set of cells that correspond to the specified footprint
//adds points to it (does not clear it beforehand) 
void EnvironmentNav3DCollisionsBase::CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, set<pair<int,int> >* footprint, const vector<sbpl_2Dpt_t>& FootprintPolygon)
{

//int pind;

#if DEBUG
//  SBPL_PRINTF("---Calculating Footprint for Pose: %f %f %f---\n",
//	 pose.x, pose.y, pose.theta);
#endif

  //handle special case where footprint is just a point
  if(FootprintPolygon.size() <= 1){
    //sbpl_2Dcell_t cell;
    //cell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETALATCfg.cellsize_m);
    //cell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETALATCfg.cellsize_m);

    footprint->insert(pair<int,int>(CONTXY2DISC(pose.x, EnvNAVXYTHETALATCfg.cellsize_m), CONTXY2DISC(pose.y, EnvNAVXYTHETALATCfg.cellsize_m)));

    /*
    for(pind = 0; pind < (int)footprint->size(); pind++)
    {
      if(cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y)
        break;
    }
    if(pind == (int)footprint->size()) footprint->push_back(cell);
    */
    return;
  }

  vector<sbpl_2Dpt_t> bounding_polygon;
  unsigned int find;
  double max_x = -INFINITECOST, min_x = INFINITECOST, max_y = -INFINITECOST, min_y = INFINITECOST;
  sbpl_2Dpt_t pt = {0,0};
  double avg_x = 0;
  double avg_y = 0;
  for(find = 0; find < FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = FootprintPolygon[find];
    
    //rotate and translate the point
    sbpl_2Dpt_t corner;
    corner.x = cos(pose.theta)*pt.x - sin(pose.theta)*pt.y + pose.x;
    corner.y = sin(pose.theta)*pt.x + cos(pose.theta)*pt.y + pose.y;
    bounding_polygon.push_back(corner);
    avg_x += corner.x;
    avg_y += corner.y;
#if DEBUG
//    SBPL_PRINTF("Pt: %f %f, Corner: %f %f\n", pt.x, pt.y, corner.x, corner.y);
#endif
    if(corner.x < min_x || find==0){
      min_x = corner.x;
    }
    if(corner.x > max_x || find==0){
      max_x = corner.x;
    }
    if(corner.y < min_y || find==0){
      min_y = corner.y;
    }
    if(corner.y > max_y || find==0){
      max_y = corner.y;
    }
    
  }
  avg_x /= bounding_polygon.size();
  avg_y /= bounding_polygon.size();

#if DEBUG
//  SBPL_PRINTF("Footprint bounding box: %f %f %f %f\n", min_x, max_x, min_y, max_y);
#endif
  //initialize previous values to something that will fail the if condition during the first iteration in the for loop
  int prev_discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETALATCfg.cellsize_m) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETALATCfg.cellsize_m) + 1;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  //TODO: this could be done as a bfs from a corner which means that each cell will only be looked at once
  //instead of many times (get rid of the cellsize/3 nonsense). Also we would only check the cells needed
  //instead of a (possibly much larger) bounding box.
  sbpl_2Dcell_t start;
  start.x = CONTXY2DISC(avg_x, EnvNAVXYTHETALATCfg.cellsize_m);
  start.y = CONTXY2DISC(avg_y, EnvNAVXYTHETALATCfg.cellsize_m);
  getFootprintCells(start, &bounding_polygon, footprint);
  /*
  for(double x=min_x; x<=max_x; x+=EnvNAVXYTHETALATCfg.cellsize_m/3){
    for(double y=min_y; y<=max_y; y+=EnvNAVXYTHETALATCfg.cellsize_m/3){
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETALATCfg.cellsize_m);
      discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETALATCfg.cellsize_m);
      
      //see if we just tested this point
      if(discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside==0){

#if DEBUG
//		SBPL_PRINTF("Testing point: %f %f Discrete: %d %d\n", pt.x, pt.y, discrete_x, discrete_y);
#endif
	
		if(IsInsideFootprint(pt, &bounding_polygon)){ 
		//convert to a grid point

#if DEBUG
//			SBPL_PRINTF("Pt Inside %f %f\n", pt.x, pt.y);
#endif

			//sbpl_2Dcell_t cell;
			//cell.x = discrete_x;
			//cell.y = discrete_y;

			//insert point if not there already
			//int pind = 0;
      //TODO: this check for existance is O(n) because it uses a vector
      // implement this as a set and it will be O(log n)
      footprint->insert(pair<int,int>(discrete_x,discrete_y));
			//for(pind = 0; pind < (int)footprint->size(); pind++)
			//{
				//if(cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y)
					//break;
			//}
			//if(pind == (int)footprint->size()) footprint->push_back(cell);

			//prev_inside = 1;

#if DEBUG
//			SBPL_PRINTF("Added pt to footprint: %f %f\n", pt.x, pt.y);
#endif
		}
		else{
			prev_inside = 0;
		}

      }
	  else
	  {
#if DEBUG
		//SBPL_PRINTF("Skipping pt: %f %f\n", pt.x, pt.y);
#endif
      }
      
      prev_discrete_x = discrete_x;
      prev_discrete_y = discrete_y;

    }//over x_min...x_max
  }
  */
}

void EnvironmentNav3DCollisionsBase::getFootprintCells(sbpl_2Dcell_t start, vector<sbpl_2Dpt_t>* polygon, set<pair<int,int> >* footprint){
  printFP = false;
  if(printFP)
    printf("pre-footprint has %d\n",footprint->size());
  int x_offset[NAVXYTHETALAT_DXYWIDTH] = {1,1,1,0,0,-1,-1,-1};
  int y_offset[NAVXYTHETALAT_DXYWIDTH] = {1,0,-1,1,-1,1,0,-1};

  int expands = 0;

  if(printFP){
    printf("start (%d %d)\n",start.x,start.y);
    for(int i=0; i<polygon->size(); i++)
      printf("polygon (%f %f)\n",polygon->at(i).x,polygon->at(i).y);
  }

  set<pair<int,int> > closed;
  footprint_fifo->clear();
  footprint_fifo->insert(start.x,start.y);
  footprint->insert(pair<int,int>(start.x,start.y));
  while(!footprint_fifo->empty()){
    int x,y;
    footprint_fifo->remove(&x,&y);
    expands++;
    if(printFP)
      printf("expanding %d %d\n",x,y);
    sbpl_2Dpt_t pt;
    for(int i=0; i<NAVXYTHETALAT_DXYWIDTH; i++){
      int new_x = x + x_offset[i];
      int new_y = y + y_offset[i];
      pt.x = DISCXY2CONT(new_x, EnvNAVXYTHETALATCfg.cellsize_m);
      pt.y = DISCXY2CONT(new_y, EnvNAVXYTHETALATCfg.cellsize_m);
      pair<int,int> p(new_x,new_y);
      if(printFP)
        printf("  generating %d %d\n",new_x,new_y);
      if(closed.find(p) == closed.end()){
        if(printFP)
          printf("  not closed\n");
        if(IsInsideFootprint(pt, polygon)){
          if(printFP)
            printf("  valid\n");
          closed.insert(p);
          footprint_fifo->insert(new_x,new_y);
          footprint->insert(p);
        }
      }
    }
  }
  if(printFP){
    printf("we had %d expands\n",expands);
    printf("post-footprint has %d\n",footprint->size());
  }
}



//calculates a set of cells that correspond to the footprint of the base
//adds points to it (does not clear it beforehand) 
void EnvironmentNav3DCollisionsBase::CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint)
{  
  set<pair<int,int> > cells;
  for(unsigned int i=0; i<footprint->size(); i++)
    cells.insert(pair<int,int>(footprint->at(i).x,footprint->at(i).y));
	//CalculateFootprintForPose(pose, footprint, EnvNAVXYTHETALATCfg.FootprintPolygon);
	CalculateFootprintForPose(pose, &cells, EnvNAVXYTHETALATCfg.FootprintPolygon);
  for(set<pair<int,int> >::iterator it=cells.begin(); it!=cells.end(); it++){
    sbpl_2Dcell_t cell;
    cell.x = it->first;
    cell.y = it->second;
    footprint->push_back(cell);
  }
}
void EnvironmentNav3DCollisionsBase::CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, set<pair<int,int> >* footprint)
{  
	CalculateFootprintForPose(pose, footprint, EnvNAVXYTHETALATCfg.FootprintPolygon);
}

//removes a set of cells that correspond to the specified footprint at the sourcepose
//adds points to it (does not clear it beforehand) 
void EnvironmentNav3DCollisionsBase::RemoveSourceFootprint(EnvNAVXYTHETALAT3Dpt_t sourcepose, vector<sbpl_2Dcell_t>* footprint, const vector<sbpl_2Dpt_t>& FootprintPolygon)
{  

	//compute source footprint
  set<pair<int,int> > cells;
	CalculateFootprintForPose(sourcepose, &cells, FootprintPolygon);
	vector<sbpl_2Dcell_t> sourcefootprint;
  sourcefootprint.reserve(cells.size());
  for(set<pair<int,int> >::iterator it=cells.begin(); it!=cells.end(); it++){
    sbpl_2Dcell_t cell;
    cell.x = it->first;
    cell.y = it->second;
    sourcefootprint.push_back(cell);
  }

	//now remove the source cells from the footprint
	for(int sind = 0; sind < (int)sourcefootprint.size(); sind++)
	{
		for(int find = 0; find < (int)footprint->size(); find++)
		{
			if(sourcefootprint.at(sind).x == footprint->at(find).x && sourcefootprint.at(sind).y == footprint->at(find).y)
			{
				footprint->erase(footprint->begin() + find);
				break;
			}
		}//over footprint
	}//over source



}

//removes a set of cells that correspond to the footprint of the base at the sourcepose
//adds points to it (does not clear it beforehand) 
void EnvironmentNav3DCollisionsBase::RemoveSourceFootprint(EnvNAVXYTHETALAT3Dpt_t sourcepose, vector<sbpl_2Dcell_t>* footprint)
{  
	RemoveSourceFootprint(sourcepose, footprint, EnvNAVXYTHETALATCfg.FootprintPolygon);
}


//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------


void EnvironmentNav3DCollisionsBase::EnsureHeuristicsUpdated(bool bGoalHeuristics)
{

	if(bNeedtoRecomputeStartHeuristics && !bGoalHeuristics)
	{
		grid2Dsearchfromstart->search(EnvNAVXYTHETALATCfg.Grid2D, EnvNAVXYTHETALATCfg.cost_inscribed_thresh, 
			EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, 
			SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH); 
		bNeedtoRecomputeStartHeuristics = false;
		SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c)
			/EnvNAVXYTHETALATCfg.nominalvel_mpersecs));

	}


	if(bNeedtoRecomputeGoalHeuristics && bGoalHeuristics)
	{
		grid2Dsearchfromgoal->search(EnvNAVXYTHETALATCfg.Grid2D, EnvNAVXYTHETALATCfg.cost_inscribed_thresh, 
			EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c,  
			SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH); 
		bNeedtoRecomputeGoalHeuristics = false;
		SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c)
			/EnvNAVXYTHETALATCfg.nominalvel_mpersecs));

	}


}



void EnvironmentNav3DCollisionsBase::ComputeHeuristicValues()
{
	//whatever necessary pre-computation of heuristic values is done here 
	SBPL_PRINTF("Precomputing heuristics...\n");
	
	//allocated 2D grid searches
	grid2Dsearchfromstart = new SBPL2DGridSearch(EnvNAVXYTHETALATCfg.EnvWidth_c, EnvNAVXYTHETALATCfg.EnvHeight_c, (float)EnvNAVXYTHETALATCfg.cellsize_m);
	grid2Dsearchfromgoal = new SBPL2DGridSearch(EnvNAVXYTHETALATCfg.EnvWidth_c, EnvNAVXYTHETALATCfg.EnvHeight_c, (float)EnvNAVXYTHETALATCfg.cellsize_m); 

	//set OPEN type to sliding buckets
	grid2Dsearchfromstart->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS); 
	grid2Dsearchfromgoal->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);

	SBPL_PRINTF("done\n");

}

//------------debugging functions---------------------------------------------
bool EnvironmentNav3DCollisionsBase::CheckQuant(FILE* fOut)
{

  for(double theta  = -10; theta < 10; theta += 2.0*PI_CONST/NAVXYTHETALAT_THETADIRS*0.01)
    {
		int nTheta = ContTheta2Disc(theta, NAVXYTHETALAT_THETADIRS);
		double newTheta = DiscTheta2Cont(nTheta, NAVXYTHETALAT_THETADIRS);
		int nnewTheta = ContTheta2Disc(newTheta, NAVXYTHETALAT_THETADIRS);

		SBPL_FPRINTF(fOut, "theta=%f(%f)->%d->%f->%d\n", theta, theta*180/PI_CONST, nTheta, newTheta, nnewTheta);

        if(nTheta != nnewTheta)
        {
            SBPL_ERROR("ERROR: invalid quantization\n");                     
            return false;
        }
    }

  return true;
}



//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------

bool EnvironmentNav3DCollisionsBase::InitializeEnv(int width, int height,
		double origin_x, double origin_y,
		double res, const unsigned char* mapdata,
		const vector<std::string>& footprint_links, const motion_planning_msgs::RobotState& robot_state,
					double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
					unsigned char obsthresh,  const char* sMotPrimFile, vector<sbpl_2Dpt_t> base_fp, vector<sbpl_2Dpt_t>* new_fp)
{

//	SBPL_PRINTF("env: initialize with width=%d height=%d start=%.3f %.3f %.3f goalx=%.3f %.3f %.3f cellsize=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d\n",
//		width, height, startx, starty, starttheta, goalx, goaly, goaltheta, cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, obsthresh);
//
//	SBPL_PRINTF("perimeter has size=%d\n", (unsigned int)perimeterptsV.size());
//
//	for(int i = 0; i < (int)perimeterptsV.size(); i++)
//	{
//		SBPL_PRINTF("perimeter(%d) = %.4f %.4f\n", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
//	}

	//m_collisionModel = collisionModel;


	EnvNAVXYTHETALATCfg.obsthresh = obsthresh;
	m_footprintLinkNames = footprint_links;
  EnvNAVXYTHETALATCfg.FootprintPolygon = base_fp;

	// set config:
	SetConfiguration(width, height,mapdata,
						0,0,0, 0,0,0,
						origin_x, origin_y,
						res, nominalvel_mpersecs, timetoturn45degsinplace_secs);

	// update robot kinematic state and footprint
	updateKinematicState(robot_state,new_fp);

	if(sMotPrimFile != NULL)
	{
		FILE* fMotPrim = fopen(sMotPrimFile, "r");
		if(fMotPrim == NULL)
		{
			SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
			throw new SBPL_Exception();
		}

		if(ReadMotionPrimitives(fMotPrim) == false)
		{
			SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
			throw new SBPL_Exception();
		}
    fclose(fMotPrim);
	}

	if(EnvNAVXYTHETALATCfg.mprimV.size() != 0)
	{
		InitGeneral(&EnvNAVXYTHETALATCfg.mprimV);
	}
	else
		InitGeneral(NULL);

  printFootprint("base.txt",EnvNAVXYTHETALATCfg.ActionsV[0][0].intersectingcellsV);
	return true;
}


bool EnvironmentNav3DCollisionsBase::InitGeneral(vector<SBPL_xytheta_mprimitive>* motionprimitiveV) {


  //Initialize other parameters of the environment
  InitializeEnvConfig(motionprimitiveV);
  
  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvironmentNav3DCollisionsBase::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = goalstateid;
	MDPCfg->startstateid = startstateid;

	return true;
}


void EnvironmentNav3DCollisionsBase::PrintHeuristicValues()
{
#ifndef ROS
  const char* heur = "heur.txt";
#endif
	FILE* fHeur = SBPL_FOPEN(heur, "w");
  if(fHeur == NULL){
    SBPL_ERROR("ERROR: could not open debug file to write heuristic\n");
    throw new SBPL_Exception();
  }
	SBPL2DGridSearch* grid2Dsearch = NULL;
	
	for(int i = 0; i < 2; i++)
	{
		if(i == 0 && grid2Dsearchfromstart != NULL)
		{
			grid2Dsearch = grid2Dsearchfromstart;
			SBPL_FPRINTF(fHeur, "start heuristics:\n");
		}
		else if(i == 1 && grid2Dsearchfromgoal != NULL)
		{
			grid2Dsearch = grid2Dsearchfromgoal;
			SBPL_FPRINTF(fHeur, "goal heuristics:\n");
		}
		else
			continue;

		for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
			for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
				if(grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y) < INFINITECOST)
					SBPL_FPRINTF(fHeur, "%5d ", grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y));
			else
				SBPL_FPRINTF(fHeur, "XXXXX ");
			}
			SBPL_FPRINTF(fHeur, "\n");
		}
	}
	SBPL_FCLOSE(fHeur);
}




void EnvironmentNav3DCollisionsBase::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
	SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: SetAllPreds is undefined\n");
	throw new SBPL_Exception();
}


void EnvironmentNav3DCollisionsBase::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
	GetSuccs(SourceStateID, SuccIDV, CostV, NULL);
}





const EnvNav3DCollisionsConfig_t* EnvironmentNav3DCollisionsBase::GetEnvNavConfig() {
  return &EnvNAVXYTHETALATCfg;
}



bool EnvironmentNav3DCollisionsBase::UpdateCost(int x, int y, unsigned char newcost)
{

#if DEBUG
  //SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,EnvNAVXYTHETALATCfg.Grid2D[x][y], newcost);
#endif

    EnvNAVXYTHETALATCfg.Grid2D[x][y] = newcost;

	bNeedtoRecomputeStartHeuristics = true;
	bNeedtoRecomputeGoalHeuristics = true;

    return true;
}


bool EnvironmentNav3DCollisionsBase::SetMap(const unsigned char* mapdata)
{
	int xind=-1, yind=-1;

	for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
		for(yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
			EnvNAVXYTHETALATCfg.Grid2D[xind][yind] = mapdata[xind+yind*EnvNAVXYTHETALATCfg.EnvWidth_c];
		}
	}

	bNeedtoRecomputeStartHeuristics = true;
	bNeedtoRecomputeGoalHeuristics = true;

	return true;

}



void EnvironmentNav3DCollisionsBase::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out EnvNAVXYTHETALAT. configuration
	
	SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: PrintEnv_Config is undefined\n");
	throw new SBPL_Exception();

}

void EnvironmentNav3DCollisionsBase::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
    SBPL_FPRINTF(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
            time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
            time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}



bool EnvironmentNav3DCollisionsBase::IsObstacle(int x, int y)
{

#if DEBUG
	SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,EnvNAVXYTHETALATCfg.Grid2D[x][y]);
#endif


	return (EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.obsthresh); 

}

void EnvironmentNav3DCollisionsBase::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double*starttheta, double* goalx, double* goaly, double* goaltheta,
									  	double* cellsize_m, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs, unsigned char* obsthresh,
										vector<SBPL_xytheta_mprimitive>* mprimitiveV)
{
	*size_x = EnvNAVXYTHETALATCfg.EnvWidth_c;
	*size_y = EnvNAVXYTHETALATCfg.EnvHeight_c;

	*startx = DISCXY2CONT(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.cellsize_m);
	*starty = DISCXY2CONT(EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.cellsize_m);
	*starttheta = DiscTheta2Cont(EnvNAVXYTHETALATCfg.StartTheta, NAVXYTHETALAT_THETADIRS);
	*goalx = DISCXY2CONT(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.cellsize_m);
	*goaly = DISCXY2CONT(EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.cellsize_m);
	*goaltheta = DiscTheta2Cont(EnvNAVXYTHETALATCfg.EndTheta, NAVXYTHETALAT_THETADIRS);;

	*cellsize_m = EnvNAVXYTHETALATCfg.cellsize_m;
	*nominalvel_mpersecs = EnvNAVXYTHETALATCfg.nominalvel_mpersecs;
	*timetoturn45degsinplace_secs = EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs;

	*obsthresh = EnvNAVXYTHETALATCfg.obsthresh;

	*mprimitiveV = EnvNAVXYTHETALATCfg.mprimV;
}


bool EnvironmentNav3DCollisionsBase::PoseContToDisc(double px, double py, double pth,
					 int &ix, int &iy, int &ith) const
{
  ix = CONTXY2DISC(px, EnvNAVXYTHETALATCfg.cellsize_m);
  iy = CONTXY2DISC(py, EnvNAVXYTHETALATCfg.cellsize_m);
  ith = ContTheta2Disc(pth, NAVXYTHETALAT_THETADIRS); // ContTheta2Disc() normalizes the angle
  return (pth >= -2*PI_CONST) && (pth <= 2*PI_CONST)
    && (ix >= 0) && (ix < EnvNAVXYTHETALATCfg.EnvWidth_c)
    && (iy >= 0) && (iy < EnvNAVXYTHETALATCfg.EnvHeight_c);
}


bool EnvironmentNav3DCollisionsBase::PoseDiscToCont(int ix, int iy, int ith,
					 double &px, double &py, double &pth) const
{
  px = DISCXY2CONT(ix, EnvNAVXYTHETALATCfg.cellsize_m);
  py = DISCXY2CONT(iy, EnvNAVXYTHETALATCfg.cellsize_m);
  pth = normalizeAngle(DiscTheta2Cont(ith, NAVXYTHETALAT_THETADIRS));
  return (ith >= 0) && (ith < NAVXYTHETALAT_THETADIRS)
    && (ix >= 0) && (ix < EnvNAVXYTHETALATCfg.EnvWidth_c)
    && (iy >= 0) && (iy < EnvNAVXYTHETALATCfg.EnvHeight_c);
}

unsigned char EnvironmentNav3DCollisionsBase::GetMapCost(int x, int y)
{
	return EnvNAVXYTHETALATCfg.Grid2D[x][y];
}



bool EnvironmentNav3DCollisionsBase::SetEnvParameter(const char* parameter, int value)
{

	if(bInitialized == true)
	{
		SBPL_ERROR("ERROR: all parameters must be set before initialization of the environment\n");
		return false;
	}

	SBPL_PRINTF("setting parameter %s to %d\n", parameter, value);

	if(strcmp(parameter, "cost_inscribed_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		EnvNAVXYTHETALATCfg.cost_inscribed_thresh = (unsigned char)value;
	}
	else if(strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = value;
	}
	else if(strcmp(parameter, "cost_obsthresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		EnvNAVXYTHETALATCfg.obsthresh = (unsigned char)value;
	}
	else
	{
		SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
		return false;
	}

	return true;
}

int EnvironmentNav3DCollisionsBase::GetEnvParameter(const char* parameter)
{

	if(strcmp(parameter, "cost_inscribed_thresh") == 0)
	{
		return (int) EnvNAVXYTHETALATCfg.cost_inscribed_thresh;
	}
	else if(strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0)
	{
		return (int) EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh;
	}
	else if(strcmp(parameter, "cost_obsthresh") == 0)
	{
		return (int) EnvNAVXYTHETALATCfg.obsthresh;
	}
	else
	{
		SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
		throw new SBPL_Exception();
	}

}

//------------------------------------------------------------------------------



//-----------------XYTHETA Enivornment (child) class---------------------------

EnvironmentNav3DCollisions::~EnvironmentNav3DCollisions()
{
	SBPL_PRINTF("destroying XYTHETALAT\n");

	//delete the states themselves first
	for (int i = 0; i < (int)StateID2CoordTable.size(); i++)
	{
		delete StateID2CoordTable.at(i);
		StateID2CoordTable.at(i) = NULL;
	}
	StateID2CoordTable.clear();

	//delete hashtable
	if(Coord2StateIDHashTable != NULL)
	{
		delete [] Coord2StateIDHashTable;
		Coord2StateIDHashTable = NULL;
	}	
	if(Coord2StateIDHashTable_lookup != NULL)
	{
		delete [] Coord2StateIDHashTable_lookup;
		Coord2StateIDHashTable_lookup = NULL;
	}

}


void EnvironmentNav3DCollisions::GetCoordFromState(int stateID, int& x, int& y, int& theta) const {
  EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
}

int EnvironmentNav3DCollisions::GetStateFromCoord(int x, int y, int theta) {

   EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL){
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }
    return OutHashEntry->stateID;
}

void EnvironmentNav3DCollisions::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<EnvNAVXYTHETALAT3Dpt_t>* xythetaPath)
{
  footPointsPub.publish(cloud_3d_collisions);
	vector<EnvNAVXYTHETALATAction_t*> actionV;
	vector<int> CostV;
	vector<int> SuccIDV;
	int targetx_c, targety_c, targettheta_c;
	int sourcex_c, sourcey_c, sourcetheta_c;

	SBPL_PRINTF("checks=%d\n", m_num2DCollChecks);

	xythetaPath->clear();

#if DEBUG
	SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

	for(int pind = 0; pind < (int)(stateIDPath->size())-1; pind++)
	{
		int sourceID = stateIDPath->at(pind);
		int targetID = stateIDPath->at(pind+1);

#if DEBUG
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
#endif


		//get successors and pick the target via the cheapest action
		SuccIDV.clear();
		CostV.clear();
		actionV.clear();
		GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);
		
		int bestcost = INFINITECOST;
		int bestsind = -1;

#if DEBUG
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
		GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
		SBPL_FPRINTF(fDeb, "looking for %d %d %d -> %d %d %d (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcetheta_c,
					targetx_c, targety_c, targettheta_c, SuccIDV.size()); 

#endif

		for(int sind = 0; sind < (int)SuccIDV.size(); sind++)
		{

#if DEBUG
		int x_c, y_c, theta_c;
		GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c);
		SBPL_FPRINTF(fDeb, "succ: %d %d %d\n", x_c, y_c, theta_c); 
#endif

			if(SuccIDV[sind] == targetID && CostV[sind] <= bestcost)
			{
				bestcost = CostV[sind];
				bestsind = sind;
			}
		}
		if(bestsind == -1)
		{
			SBPL_ERROR("ERROR: successor not found for transition:\n");
			GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
			GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
			SBPL_PRINTF("%d %d %d -> %d %d %d\n", sourcex_c, sourcey_c, sourcetheta_c,
					targetx_c, targety_c, targettheta_c); 
			throw new SBPL_Exception();
		}

    /*
    always3Dcheck = true;
    vector<EnvNAVXYTHETALATAction_t*> actionV2;
    vector<int> CostV2;
    vector<int> SuccIDV2;
		GetSuccs(sourceID, &SuccIDV2, &CostV2, &actionV2);
    always3Dcheck = false;
    bool foundit = false;
    for(int i=0; i<actionV2.size(); i++)
      if(actionV[bestsind]==actionV2[i])
        foundit = true;
    if(!foundit){
      printf("we have a problem...\n");
      int sourcex_c, sourcey_c, sourcetheta_c;
      GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
      drawStuff(actionV[bestsind],sourcex_c, sourcey_c);
    }
    */

		//now push in the actual path
		int sourcex_c, sourcey_c, sourcetheta_c;
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
		double sourcex, sourcey;
		sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETALATCfg.cellsize_m);
		//TODO - when there are no motion primitives we should still print source state
		for(int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size())-1; ipind++) 
		{
			//translate appropriately
			EnvNAVXYTHETALAT3Dpt_t intermpt = actionV[bestsind]->intermptV[ipind];
			intermpt.x += sourcex;
			intermpt.y += sourcey;

#if DEBUG
			int nx = CONTXY2DISC(intermpt.x, EnvNAVXYTHETALATCfg.cellsize_m);
			int ny = CONTXY2DISC(intermpt.y, EnvNAVXYTHETALATCfg.cellsize_m);
			SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ", 
				intermpt.x, intermpt.y, intermpt.theta, 
				nx, ny, 
				ContTheta2Disc(intermpt.theta, NAVXYTHETALAT_THETADIRS), EnvNAVXYTHETALATCfg.Grid2D[nx][ny]);
			if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
			else SBPL_FPRINTF(fDeb, "\n");
#endif

			//store
			xythetaPath->push_back(intermpt);
		}
	}
}


// goal position in costmap coordinates!
// returns the stateid if success, and -1 otherwise
int EnvironmentNav3DCollisions::SetGoal(double x_m, double y_m, double theta_rad){

	int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int theta = ContTheta2Disc(theta_rad, NAVXYTHETALAT_THETADIRS);

	SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

	if(!IsWithinMapCell(x,y))
	{
		SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x,y);
		return -1;
	}

    if(!IsValidConfiguration(x,y,theta))
	{
    	//ROS_INFO("Goal configuration  %f %f %f in 2D collision, checking 3D", x_m, y_m, theta_rad);
    	//if (isIn3DCollision(x_m,y_m,theta_rad)){
    		ROS_ERROR("Goal configuration %f %f %f in 3D collision", x_m, y_m, theta_rad);
    		visualize3DCollsisions();
    		return -1;
    	//}

	}


    visualization_msgs::MarkerArray arr;
    std_msgs::ColorRGBA col;
    col.g = 1.0;
    col.a = 0.5;
    updateRobotPosition(x_m, y_m, theta_rad);
    //m_planningCollisionModel.getRobotMarkersGivenState(*m_kinematicState, arr, col, "robot_goal", ros::Duration(0));
    col.b = 1.0;
    m_planningCollisionModel.getAttachedCollisionObjectMarkers(*m_kinematicState, arr, "attached", col, ros::Duration(0.0));
    m_collisionMarkerPub.publish(arr);


    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL){
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }

	//need to recompute start heuristics?
	if(goalstateid != OutHashEntry->stateID)
	{
		bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
		bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
	}



    goalstateid = OutHashEntry->stateID;

	EnvNAVXYTHETALATCfg.EndX_c = x;
	EnvNAVXYTHETALATCfg.EndY_c = y;
	EnvNAVXYTHETALATCfg.EndTheta = theta;


    return goalstateid;

}

void EnvironmentNav3DCollisions::drawPose(double x, double y, double theta, int i){
    visualization_msgs::MarkerArray arr;
    std_msgs::ColorRGBA col;
    col.r = 0.0;
    col.g = 1.0;
    col.b = 1.0;
    col.a = 1.0;
    updateRobotPosition(x, y, theta);
    char buf[32];
    sprintf(buf,"robot_goal%d",i);
    m_planningCollisionModel.getRobotMarkersGivenState(*m_kinematicState, arr, col, buf, ros::Duration(0));
    //col.b = 1.0;
    //m_planningCollisionModel.getAttachedCollisionObjectMarkers(*m_kinematicState, arr, "attached", col, ros::Duration(0.0));
    m_collisionMarkerPub.publish(arr);
}

// Start postion in costmap frame!
//returns the stateid if success, and -1 otherwise

int EnvironmentNav3DCollisions::SetStart(double x_m, double y_m, double theta_rad){
  succ_count = 0;
  center_count = 0;
  footprint_count = 0;
  mesh_count = 0;

	int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int theta = ContTheta2Disc(theta_rad, NAVXYTHETALAT_THETADIRS);

	if(!IsWithinMapCell(x,y))
	{
		SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x,y);
		return -1;
	}

	SBPL_PRINTF("env: setting start to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

    if(!IsValidConfiguration(x,y,theta))
	{
    	//ROS_INFO("Start configuration in 2D collision, checking 3D");
    	//if (isIn3DCollision(x_m, y_m, theta_rad)){
    		ROS_ERROR("Start configuration %f %f %f in 3D collision", x_m, y_m, theta_rad);
    		visualize3DCollsisions();
    		return -1;
    	//}
	}

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL){
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }

	//need to recompute start heuristics?
	if(startstateid != OutHashEntry->stateID)
	{
		bNeedtoRecomputeStartHeuristics = true;
		bNeedtoRecomputeGoalHeuristics = true; //because termination condition can be not all states TODO - make it dependent on term. condition
	}

	//set start
    startstateid = OutHashEntry->stateID;
	EnvNAVXYTHETALATCfg.StartX_c = x;
	EnvNAVXYTHETALATCfg.StartY_c = y;
	EnvNAVXYTHETALATCfg.StartTheta = theta;

    return startstateid;

}

void EnvironmentNav3DCollisions::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal (2)\n");
		throw new SBPL_Exception();
	}
#endif

	if(fOut == NULL)
		fOut = stdout;

	EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];

	if(stateID == goalstateid && bVerbose)
	{
		SBPL_FPRINTF(fOut, "the state is a goal state\n");
	}

    if(bVerbose)
    	SBPL_FPRINTF(fOut, "X=%d Y=%d Theta=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);
    else
    	SBPL_FPRINTF(fOut, "%.3f %.3f %.3f\n", DISCXY2CONT(HashEntry->X, EnvNAVXYTHETALATCfg.cellsize_m), DISCXY2CONT(HashEntry->Y,EnvNAVXYTHETALATCfg.cellsize_m), 
		DiscTheta2Cont(HashEntry->Theta, NAVXYTHETALAT_THETADIRS));

}


EnvNAVXYTHETALATHashEntry_t* EnvironmentNav3DCollisions::GetHashEntry_lookup(int X, int Y, int Theta)
{

	int index = XYTHETA2INDEX(X,Y,Theta);	
	return Coord2StateIDHashTable_lookup[index];

}


EnvNAVXYTHETALATHashEntry_t* EnvironmentNav3DCollisions::GetHashEntry_hash(int X, int Y, int Theta)
{

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	int binid = GETHASHBIN(X, Y, Theta);	

#if DEBUG
	if ((int)Coord2StateIDHashTable[binid].size() > 5)
	{
		SBPL_FPRINTF(fDeb, "WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
			binid, X, Y, Coord2StateIDHashTable[binid].size());
		
		PrintHashTableHist(fDeb);		
	}
#endif

	//iterate over the states in the bin and select the perfect match
	vector<EnvNAVXYTHETALATHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
	for(int ind = 0; ind < (int)binV->size(); ind++)
	{
		EnvNAVXYTHETALATHashEntry_t* hashentry = binV->at(ind);
		if( hashentry->X == X  && hashentry->Y == Y && hashentry->Theta == Theta)
		{
#if TIME_DEBUG
			time_gethash += clock()-currenttime;
#endif
			return hashentry;
		}
	}

#if TIME_DEBUG	
	time_gethash += clock()-currenttime;
#endif

	return NULL;	  
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNav3DCollisions::CreateNewHashEntry_lookup(int X, int Y, int Theta)
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAVXYTHETALATHashEntry_t* HashEntry = new EnvNAVXYTHETALATHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Theta = Theta;
	HashEntry->iteration = 0;

	HashEntry->stateID = StateID2CoordTable.size();

	//insert into the tables
	StateID2CoordTable.push_back(HashEntry);

	int index = XYTHETA2INDEX(X,Y,Theta);

#if DEBUG
	if(Coord2StateIDHashTable_lookup[index] != NULL)
	{
		SBPL_ERROR("ERROR: creating hash entry for non-NULL hashentry\n");
		throw new SBPL_Exception();
	}
#endif

	Coord2StateIDHashTable_lookup[index] = 	HashEntry;

	//insert into and initialize the mappings
	int* entry = new int [NUMOFINDICES_STATEID2IND]; 
	StateID2IndexMapping.push_back(entry);
	for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
	{
		StateID2IndexMapping[HashEntry->stateID][i] = -1;
	}

	if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
	{
		SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
		throw new SBPL_Exception();	
	}

#if TIME_DEBUG
	time_createhash += clock()-currenttime;
#endif

	return HashEntry;
}




EnvNAVXYTHETALATHashEntry_t* EnvironmentNav3DCollisions::CreateNewHashEntry_hash(int X, int Y, int Theta)
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAVXYTHETALATHashEntry_t* HashEntry = new EnvNAVXYTHETALATHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Theta = Theta;
	HashEntry->iteration = 0;

	HashEntry->stateID = StateID2CoordTable.size();

	//insert into the tables
	StateID2CoordTable.push_back(HashEntry);


	//get the hash table bin
	i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta); 

	//insert the entry into the bin
    Coord2StateIDHashTable[i].push_back(HashEntry);

	//insert into and initialize the mappings
	int* entry = new int [NUMOFINDICES_STATEID2IND]; 
	StateID2IndexMapping.push_back(entry);
	for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
	{
		StateID2IndexMapping[HashEntry->stateID][i] = -1;
	}

	if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
	{
		SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
		throw new SBPL_Exception();	
	}

#if TIME_DEBUG
	time_createhash += clock()-currenttime;
#endif

	return HashEntry;
}



void EnvironmentNav3DCollisions::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
{
    int aind;

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETALATCfg.actionwidth); 
    CostV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
	if(actionV != NULL)
	{
		actionV->clear();
		actionV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
	}

	//goal state should be absorbing
	if(SourceStateID == goalstateid)
		return;

	//get X, Y for the state
	EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

  if(actionV!=NULL)
    printf("id:%d x:%d y:%d th:%d\n",SourceStateID,HashEntry->X,HashEntry->Y,HashEntry->Theta);
	
	//iterate through actions
	for (aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
	{
		EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, NAVXYTHETALAT_THETADIRS);	

		// this is already done in getActionCosts (also taking 3D into account)!
        //skip the invalid cells
//        if(!IsValidCell(newX, newY))
//			continue;

		//get cost
    bool collision;
		int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction, &collision);
        if(cost >= INFINITECOST)
            continue;

    	EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
		if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
		}

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
		if(actionV != NULL)
			actionV->push_back(nav3daction);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif

}

void EnvironmentNav3DCollisions::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

	//TODO- to support tolerance, need: a) generate preds for goal state based on all possible goal state variable settings,
	//b) change goal check condition in gethashentry c) change getpredsofchangedcells and getsuccsofchangedcells functions

    int aind;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	//get X, Y for the state
	EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int)HashEntry->Theta].size()); 
    CostV->reserve(EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int)HashEntry->Theta].size());
	
	//iterate through actions
	vector<EnvNAVXYTHETALATAction_t*>* actionsV = &EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int)HashEntry->Theta];
	for (aind = 0; aind < (int)EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int)HashEntry->Theta].size(); aind++)
	{

		EnvNAVXYTHETALATAction_t* nav3daction = actionsV->at(aind);

        int predX = HashEntry->X - nav3daction->dX;
		int predY = HashEntry->Y - nav3daction->dY;
		int predTheta = nav3daction->starttheta;	
	
		// this is already done in getActionCosts (also taking 3D into account)!
		//skip the invalid cells
//        if(!IsValidCell(predX, predY))
//			continue;

		//get cost
    bool collision;
		int cost = GetActionCost(predX, predY, predTheta, nav3daction, &collision);
	    if(cost >= INFINITECOST)
			continue;
        
    	EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
		if((OutHashEntry = (this->*GetHashEntry)(predX, predY, predTheta)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = (this->*CreateNewHashEntry)(predX, predY, predTheta);
		}

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif


}

void EnvironmentNav3DCollisions::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

	int cost;

#if DEBUG
	if(state->StateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in Env... function: stateID illegal\n");
		throw new SBPL_Exception();
	}

	if((int)state->Actions.size() != 0)
	{
		SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
		throw new SBPL_Exception();
	}
#endif
	

	//goal state should be absorbing
	if(state->StateID ==goalstateid)
		return;

	//get X, Y for the state
	EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];
	
	//iterate through actions
	for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
	{
		EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, NAVXYTHETALAT_THETADIRS);	

		// this is already done in getActionCosts (also taking 3D into account)!
        //skip the invalid cells
//        if(!IsValidCell(newX, newY))
//			continue;

		//get cost
    bool collision;
		cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction, &collision);
        if(cost >= INFINITECOST)
            continue;

		//add the action
		CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    	EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
		if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
		}
		action->AddOutcome(OutHashEntry->stateID, cost, 1.0); 

#if TIME_DEBUG
		time3_addallout += clock()-currenttime;
#endif

	}
}


void EnvironmentNav3DCollisions::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV)
{
	nav2dcell_t cell;
	EnvNAVXYTHETALAT3Dcell_t affectedcell;
	EnvNAVXYTHETALATHashEntry_t* affectedHashEntry;

	//increment iteration for processing savings
	iteration++;

	for(int i = 0; i < (int)changedcellsV->size(); i++) 
	{
		cell = changedcellsV->at(i);
			
		//now iterate over all states that could potentially be affected
		for(int sind = 0; sind < (int)affectedpredstatesV.size(); sind++)
		{
			affectedcell = affectedpredstatesV.at(sind);

			//translate to correct for the offset
			affectedcell.x = affectedcell.x + cell.x;
			affectedcell.y = affectedcell.y + cell.y;

			//insert only if it was actually generated
		    affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta);
			if(affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
			{
				preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
				affectedHashEntry->iteration = iteration; //mark as already inserted
			}
		}
	}
}

void EnvironmentNav3DCollisions::GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV)
{
	nav2dcell_t cell;
	EnvNAVXYTHETALAT3Dcell_t affectedcell;
	EnvNAVXYTHETALATHashEntry_t* affectedHashEntry;

	SBPL_ERROR("ERROR: getsuccs is not supported currently\n");
	throw new SBPL_Exception();

	//increment iteration for processing savings
	iteration++;

	//TODO - check
	for(int i = 0; i < (int)changedcellsV->size(); i++) 
	{
		cell = changedcellsV->at(i);
			
		//now iterate over all states that could potentially be affected
		for(int sind = 0; sind < (int)affectedsuccstatesV.size(); sind++)
		{
			affectedcell = affectedsuccstatesV.at(sind);

			//translate to correct for the offset
			affectedcell.x = affectedcell.x + cell.x;
			affectedcell.y = affectedcell.y + cell.y;

			//insert only if it was actually generated
		    affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta);
			if(affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
			{
				succs_of_changededgesIDV->push_back(affectedHashEntry->stateID);
				affectedHashEntry->iteration = iteration; //mark as already inserted
			}
		}
	}
}

void EnvironmentNav3DCollisions::InitializeEnvironment()
{
	EnvNAVXYTHETALATHashEntry_t* HashEntry;

	int maxsize = EnvNAVXYTHETALATCfg.EnvWidth_c*EnvNAVXYTHETALATCfg.EnvHeight_c*NAVXYTHETALAT_THETADIRS;

	if(maxsize <= SBPL_XYTHETALAT_MAXSTATESFORLOOKUP)
	{
		SBPL_PRINTF("environment stores states in lookup table\n");

		Coord2StateIDHashTable_lookup = new EnvNAVXYTHETALATHashEntry_t*[maxsize]; 
		for(int i = 0; i < maxsize; i++)
			Coord2StateIDHashTable_lookup[i] = NULL;
		GetHashEntry = &EnvironmentNav3DCollisions::GetHashEntry_lookup;
		CreateNewHashEntry = &EnvironmentNav3DCollisions::CreateNewHashEntry_lookup;
		
		//not using hash table
		HashTableSize = 0;
		Coord2StateIDHashTable = NULL;
	}
	else
	{		
		SBPL_PRINTF("environment stores states in hashtable\n");

		//initialize the map from Coord to StateID
		HashTableSize = 4*1024*1024; //should be power of two 
		Coord2StateIDHashTable = new vector<EnvNAVXYTHETALATHashEntry_t*>[HashTableSize]; 
		GetHashEntry = &EnvironmentNav3DCollisions::GetHashEntry_hash;
		CreateNewHashEntry = &EnvironmentNav3DCollisions::CreateNewHashEntry_hash;

		//not using hash
		Coord2StateIDHashTable_lookup = NULL;
	}


	//initialize the map from StateID to Coord
	StateID2CoordTable.clear();

	//create start state 
	if((HashEntry = (this->*GetHashEntry)(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.StartTheta)) == NULL){
        //have to create a new entry
		HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.StartTheta);
	}
	startstateid = HashEntry->stateID;

	//create goal state 
	if((HashEntry = (this->*GetHashEntry)(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.EndTheta)) == NULL){
        //have to create a new entry
		HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.EndTheta);
	}
	goalstateid = HashEntry->stateID;

	//initialized
	bInitialized = true;

}


//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentNav3DCollisions::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta)
{

	return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Theta)<<2)) & (HashTableSize-1);
}

void EnvironmentNav3DCollisions::PrintHashTableHist(FILE* fOut)
{
	int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

	for(int  j = 0; j < HashTableSize; j++)
	{
	  if((int)Coord2StateIDHashTable[j].size() == 0)
			s0++;
		else if((int)Coord2StateIDHashTable[j].size() < 5)
			s1++;
		else if((int)Coord2StateIDHashTable[j].size() < 25)
			s50++;
		else if((int)Coord2StateIDHashTable[j].size() < 50)
			s100++;
		else if((int)Coord2StateIDHashTable[j].size() < 100)
			s200++;
		else if((int)Coord2StateIDHashTable[j].size() < 400)
			s300++;
		else
			slarge++;
	}
	SBPL_FPRINTF(fOut, "hash table histogram: 0:%d, <5:%d, <25:%d, <50:%d, <100:%d, <400:%d, >400:%d\n",
		s0,s1, s50, s100, s200,s300,slarge);
}

int EnvironmentNav3DCollisions::GetFromToHeuristic(int FromStateID, int ToStateID)
{

#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)StateID2CoordTable.size() 
		|| ToStateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif

	//get X, Y for the state
	EnvNAVXYTHETALATHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
	EnvNAVXYTHETALATHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];
	
	//TODO - check if one of the gridsearches already computed and then use it.
	

	return (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y)/EnvNAVXYTHETALATCfg.nominalvel_mpersecs);	

}


int EnvironmentNav3DCollisions::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif

	EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); //computes distances from start state that is grid2D, so it is EndX_c EndY_c 
	int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EuclideanDistance_m(HashEntry->X, HashEntry->Y, EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c));
		
	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETALATCfg.nominalvel_mpersecs); 

}


int EnvironmentNav3DCollisions::GetStartHeuristic(int stateID)
{


#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif

	EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
	int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EuclideanDistance_m(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, HashEntry->X, HashEntry->Y));
		
	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETALATCfg.nominalvel_mpersecs); 

}

int EnvironmentNav3DCollisions::SizeofCreatedEnv()
{
	return (int)StateID2CoordTable.size();
	
}
//------------------------------------------------------------------------------

void EnvironmentNav3DCollisionsBase::printFootprint(char* filename, vector<sbpl_2Dcell_t> footprint){
  printf("start print\n");
  FILE* fout=fopen(filename, "w");
  for(unsigned int i=0; i<footprint.size(); i++)
    fprintf(fout,"%d %d\n",footprint[i].x,footprint[i].y);
  fclose(fout);
  printf("end print\n");
}

