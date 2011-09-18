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
#include <sbpl_lattice_planner_layer_3d/environment_navxythetamlevlat.h>
#include <planning_environment/models/model_utils.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <geometry_msgs/PolygonStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/surface/concave_hull.h>

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0; 


//-----------------constructors/destructors-------------------------------
EnvironmentNAVXYTHETAMLEVLAT3D::EnvironmentNAVXYTHETAMLEVLAT3D()
{
	numofadditionalzlevs = 0; //by default there is only base level, no additional levels
	AddLevelFootprintPolygonV = NULL;
	AdditionalInfoinActionsV = NULL; 
	AddLevelGrid2D = NULL;
	AddLevel_cost_possibly_circumscribed_thresh = NULL;
	AddLevel_cost_inscribed_thresh = NULL;

}

EnvironmentNAVXYTHETAMLEVLAT3D::~EnvironmentNAVXYTHETAMLEVLAT3D()
{
	if(AddLevelFootprintPolygonV != NULL)
	{
		delete [] AddLevelFootprintPolygonV;
		AddLevelFootprintPolygonV = NULL;
	}

	if(AdditionalInfoinActionsV != NULL)
	{
		for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
		{
			for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
			{
				delete [] AdditionalInfoinActionsV[tind][aind].intersectingcellsV;
			}
			delete [] AdditionalInfoinActionsV[tind];
		}		
		delete [] AdditionalInfoinActionsV;
		AdditionalInfoinActionsV = NULL;
	}

	if(AddLevelGrid2D != NULL)
	{
		for(int levelind = 0; levelind < numofadditionalzlevs; levelind++)
		{
			for (int xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
				delete [] AddLevelGrid2D[levelind][xind];
			}
			delete [] AddLevelGrid2D[levelind];
		}
		delete [] AddLevelGrid2D;
		AddLevelGrid2D = NULL;
	}

	if(AddLevel_cost_possibly_circumscribed_thresh != NULL)
	{
		delete [] AddLevel_cost_possibly_circumscribed_thresh;
		AddLevel_cost_possibly_circumscribed_thresh = NULL;
	}

	if(AddLevel_cost_inscribed_thresh != NULL)
	{
		delete [] AddLevel_cost_inscribed_thresh;
		AddLevel_cost_inscribed_thresh = NULL;
	}

	//reset the number of additional levels
	numofadditionalzlevs = 0;
}


//---------------------------------------------------------------------



//-------------------problem specific and local functions---------------------

//returns true if cell is traversable and within map limits - it checks against all levels including the base one
bool EnvironmentNAVXYTHETAMLEVLAT3D::IsValidCell(int X, int Y)
{
	int levelind;

	if(!EnvironmentNav3DCollisions::IsValidCell(X,Y))
		return false;

	//iterate through the additional levels
	for (levelind=0; levelind < numofadditionalzlevs; levelind++)
	{
		if(AddLevelGrid2D[levelind][X][Y] >= EnvNAVXYTHETALATCfg.obsthresh)
			return false;
	}
	//otherwise the cell is valid at all levels
	return true;
}

// returns true if cell is traversable and within map limits for a particular level
bool EnvironmentNAVXYTHETAMLEVLAT3D::IsValidCell(int X, int Y, int levind)
{
	return (X >= 0 && X < EnvNAVXYTHETALATCfg.EnvWidth_c && 
			Y >= 0 && Y < EnvNAVXYTHETALATCfg.EnvHeight_c && levind < numofadditionalzlevs &&
			AddLevelGrid2D[levind][X][Y] < EnvNAVXYTHETALATCfg.obsthresh);
}


//returns true if cell is untraversable at all levels
bool EnvironmentNAVXYTHETAMLEVLAT3D::IsObstacle(int X, int Y)
{

	int levelind;

	if(EnvironmentNav3DCollisions::IsObstacle(X,Y))
		return true;

	//iterate through the additional levels
	for (levelind=0; levelind < numofadditionalzlevs; levelind++)
	{
		if(AddLevelGrid2D[levelind][X][Y] >= EnvNAVXYTHETALATCfg.obsthresh)
			return true;
	}
	//otherwise the cell is obstacle-free at all cells
	return false;


}

//returns true if cell is untraversable in level # levelnum. If levelnum = -1, then it checks all levels
bool EnvironmentNAVXYTHETAMLEVLAT3D::IsObstacle(int X, int Y, int levind)
{
#if DEBUG
	if(levind >= numofadditionalzlevs)
	{
		SBPL_ERROR("ERROR: IsObstacle invoked at level %d\n", levind);
		SBPL_FPRINTF(fDeb, "ERROR: IsObstacle invoked at level %d\n", levind);
		return false;
	}
#endif

	return (AddLevelGrid2D[levind][X][Y] >= EnvNAVXYTHETALATCfg.obsthresh);
}

// returns the maximum over all levels of the cost corresponding to the cell <x,y>
unsigned char EnvironmentNAVXYTHETAMLEVLAT3D::GetMapCost(int X, int Y)
{
	unsigned char mapcost = EnvNAVXYTHETALATCfg.Grid2D[X][Y];

	for (int levind=0; levind < numofadditionalzlevs; levind++)
	{
		mapcost = __max(mapcost, AddLevelGrid2D[levind][X][Y]);
	}

	return mapcost;
}

// returns the cost corresponding to the cell <x,y> at level levind
unsigned char EnvironmentNAVXYTHETAMLEVLAT3D::GetMapCost(int X, int Y, int levind)
{
#if DEBUG
	if(levind >= numofadditionalzlevs)
	{
		SBPL_ERROR("ERROR: GetMapCost invoked at level %d\n", levind);
		SBPL_FPRINTF(fDeb, "ERROR: GetMapCost invoked at level %d\n", levind);
		return false;
	}
#endif

	return AddLevelGrid2D[levind][X][Y];
}

//returns false if robot intersects obstacles or lies outside of the map.
bool EnvironmentNAVXYTHETAMLEVLAT3D::IsValidConfiguration(int X, int Y, int Theta){
  printf("checking 3d valid config\n");
  return !isIn3DCollision(xyDisc2Cont(X), xyDisc2Cont(Y), thetaDisc2Cont(Theta));
}

	
int EnvironmentNAVXYTHETAMLEVLAT3D::GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action, bool* not_used){
  //printf("getting action cost for (%d %d %d) -> (%d %d %d)\n",SourceX,SourceY,SourceTheta,SourceX+action->dX,SourceY+action->dY,action->endtheta);
  bool possible_collision = false;
  m_num2DCollChecks++;

  //printf("check base\n");
  int basecost = 0;
  if(use_multi_layer){
    int basecost = EnvironmentNav3DCollisions::GetActionCost(SourceX, SourceY, SourceTheta, action, &possible_collision);
    if(basecost >= INFINITECOST){
      //printf("base returned infinite\n");
      return INFINITECOST;
    }
  }

  //printf("check layers\n");
  int addcost = GetActionCostacrossAddLevels(SourceX, SourceY, SourceTheta, action, &possible_collision);
  if(addcost >= INFINITECOST){
    //printf("layers returned infinite\n");
    return INFINITECOST;
  }

  // finally, check for 3D collisions if there were 2D ones
  if(possible_collision || always3Dcheck){
    m_num3DCollChecks++;
    if(!possible_collision)
      printf("2D says no collision, but let's check the 3D anyway...\n");
    //printf("we have a possible collision......3d check\n");
    // first, start and end of action:
    if (isIn3DCollision(xyDisc2Cont(SourceX), xyDisc2Cont(SourceY), thetaDisc2Cont(SourceTheta))
        || isIn3DCollision(xyDisc2Cont(SourceX + action->dX), xyDisc2Cont(SourceY + action->dY), thetaDisc2Cont(action->endtheta))){
      //printf("3d collision\n");
      return INFINITECOST;
    }

    // intermediate cells:
    EnvNAVXYTHETALAT3Dcell_t interm3Dcell;
    for(int i = 0; i < (int)action->interm3DcellsV.size(); i++){
      interm3Dcell = action->interm3DcellsV.at(i);
      interm3Dcell.x = interm3Dcell.x + SourceX;
      interm3Dcell.y = interm3Dcell.y + SourceY;

      if (isIn3DCollision(xyDisc2Cont(interm3Dcell.x), xyDisc2Cont(interm3Dcell.y), thetaDisc2Cont(interm3Dcell.theta))){
        //printf("3d collision\n");
        return INFINITECOST;
      }
    }
  }
	
  return __max(basecost, addcost);
}


int EnvironmentNAVXYTHETAMLEVLAT3D::GetActionCostacrossAddLevels(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action, bool* possible_collision){
	sbpl_2Dcell_t cell;
	EnvNAVXYTHETALAT3Dcell_t interm3Dcell;
	int i, levelind=-1;

	if(!IsValidCell(SourceX, SourceY)){
    //printf("layer source not valid\n");
		return INFINITECOST;
  }
	if(!IsValidCell(SourceX + action->dX, SourceY + action->dY)){
    //printf("layer sink not valid\n");
		return INFINITECOST;
  }

	//case of no levels
	if(numofadditionalzlevs == 0)
		return 0;

	//need to iterate over discretized center cells and compute cost based on them
	unsigned char maxcellcost = 0;
	unsigned char* maxcellcostateachlevel = new unsigned char [numofadditionalzlevs];
	for (levelind=0; levelind < numofadditionalzlevs; levelind++)
		maxcellcostateachlevel[levelind] = 0;


  if(use_multi_layer){
    for(i = 0; i < (int)action->interm3DcellsV.size(); i++){
      interm3Dcell = action->interm3DcellsV.at(i);
      interm3Dcell.x = interm3Dcell.x + SourceX;
      interm3Dcell.y = interm3Dcell.y + SourceY;
      
      for (levelind=0; levelind < numofadditionalzlevs; levelind++){
        maxcellcost = __max(maxcellcost, AddLevelGrid2D[levelind][interm3Dcell.x][interm3Dcell.y]);
        maxcellcostateachlevel[levelind] = __max(maxcellcostateachlevel[levelind], AddLevelGrid2D[levelind][interm3Dcell.x][interm3Dcell.y]);
      }
    }
    //printf("layer max center costs {%d %d}\n",maxcellcostateachlevel[0],maxcellcostateachlevel[1]);

    //check collisions that for the particular footprint orientation along the action
    for (levelind=0; levelind < numofadditionalzlevs; levelind++){
      if(AddLevelFootprintPolygonV[levelind].size() > 1 && 
        (int)maxcellcostateachlevel[levelind] >= AddLevel_cost_possibly_circumscribed_thresh[levelind])
      {
        //checks++;

        //m_num2DCollChecks++;
        //get intersecting cells for this level
        vector<sbpl_2Dcell_t>* intersectingcellsV = &AdditionalInfoinActionsV[(unsigned int)action->starttheta][action->aind].intersectingcellsV[levelind]; 
        for(i = 0; i < (int)intersectingcellsV->size(); i++) 
        {
          //get the cell in the map
          cell = intersectingcellsV->at(i);
          cell.x = cell.x + SourceX;
          cell.y = cell.y + SourceY;
          
          if(cell.x < 0 || cell.x >= EnvNAVXYTHETALATCfg.EnvWidth_c || cell.y < 0 || cell.y >= EnvNAVXYTHETALATCfg.EnvHeight_c){
            //printf("layer %d off map\n",levelind);
            return INFINITECOST;
          }
          //printf("  checking (%d,%d) with cost of %d\n",cell.x,cell.y,AddLevelGrid2D[levelind][cell.x][cell.y]);
          if(AddLevelGrid2D[levelind][cell.x][cell.y] > EnvNAVXYTHETALATCfg.obsthresh){
            //printf("layer %d tall obs\n",levelind);
            return INFINITECOST;
          }
          if(AddLevelGrid2D[levelind][cell.x][cell.y] == EnvNAVXYTHETALATCfg.obsthresh){
            //printf("layer %d short obs\n",levelind);
            *possible_collision = true;

            pcl::PointXYZ pt;
            pt.x = DISCXY2CONT(SourceX,EnvNAVXYTHETALATCfg.cellsize_m) - 12.487500;
            pt.y = DISCXY2CONT(SourceY,EnvNAVXYTHETALATCfg.cellsize_m) - 12.487500;
            pt.z = 0.05 + action->starttheta*0.05;
            cloud_3d_collisions->points.push_back(pt);
          }
        }
      }
    }
  }
  else{
    levelind = 0;
    for(i = 0; i < (int)action->interm3DcellsV.size(); i++){
      interm3Dcell = action->interm3DcellsV.at(i);
      interm3Dcell.x = interm3Dcell.x + SourceX;
      interm3Dcell.y = interm3Dcell.y + SourceY;
      
      maxcellcost = __max(maxcellcost, AddLevelGrid2D[levelind][interm3Dcell.x][interm3Dcell.y]);
      maxcellcostateachlevel[levelind] = __max(maxcellcostateachlevel[levelind], AddLevelGrid2D[levelind][interm3Dcell.x][interm3Dcell.y]);
    }
    //printf("layer max center costs {%d %d}\n",maxcellcostateachlevel[0],maxcellcostateachlevel[1]);

    //check collisions that for the particular footprint orientation along the action
    if(AddLevelFootprintPolygonV[levelind].size() > 1 && 
      (int)maxcellcostateachlevel[levelind] >= AddLevel_cost_possibly_circumscribed_thresh[levelind])
    {
      //checks++;

      //m_num2DCollChecks++;
      //get intersecting cells for this level
      vector<sbpl_2Dcell_t>* intersectingcellsV = &AdditionalInfoinActionsV[(unsigned int)action->starttheta][action->aind].intersectingcellsV[levelind]; 
      for(i = 0; i < (int)intersectingcellsV->size(); i++) 
      {
        //get the cell in the map
        cell = intersectingcellsV->at(i);
        cell.x = cell.x + SourceX;
        cell.y = cell.y + SourceY;
        
        if(cell.x < 0 || cell.x >= EnvNAVXYTHETALATCfg.EnvWidth_c || cell.y < 0 || cell.y >= EnvNAVXYTHETALATCfg.EnvHeight_c){
          //printf("layer %d off map\n",levelind);
          return INFINITECOST;
        }
        //printf("  checking (%d,%d) with cost of %d\n",cell.x,cell.y,AddLevelGrid2D[levelind][cell.x][cell.y]);
        if(AddLevelGrid2D[levelind][cell.x][cell.y] > EnvNAVXYTHETALATCfg.obsthresh){
          //printf("layer %d tall obs\n",levelind);
          return INFINITECOST;
        }
        if(AddLevelGrid2D[levelind][cell.x][cell.y] == EnvNAVXYTHETALATCfg.obsthresh){
          //return INFINITECOST;
          //printf("layer %d short obs\n",levelind);
          *possible_collision = true;

          pcl::PointXYZ pt;
          pt.x = DISCXY2CONT(SourceX,EnvNAVXYTHETALATCfg.cellsize_m) - 12.487500;
          pt.y = DISCXY2CONT(SourceY,EnvNAVXYTHETALATCfg.cellsize_m) - 12.487500;
          pt.z = 0.05 + action->starttheta*0.05;
          cloud_3d_collisions->points.push_back(pt);
        }
      }
    }
  }
  //printf("motion valid with cost %d\n",maxcellcost);

	//no need to max it with grid2D to ensure consistency of h2D since it is done for the base class

	//clean up
	delete [] maxcellcostateachlevel;

  return action->cost*(((int)maxcellcost)+1); //use cell cost as multiplicative factor
}

//---------------------------------------------------------------------


//------------debugging functions---------------------------------------------


//-----------------------------------------------------------------------------


//-----------interface with outside functions-----------------------------------
 /*
 initialization of additional levels. 0 is the original one. All additional ones will start with index 1
 */
bool EnvironmentNAVXYTHETAMLEVLAT3D::InitializeAdditionalLevels(int numofadditionalzlevs_in, 
                                                                const vector<sbpl_2Dpt_t>* perimeterptsV,
                                                                unsigned char* cost_inscribed_thresh_in,
                                                                unsigned char* cost_possibly_circumscribed_thresh_in)
{
	int levelind = -1, xind=-1, yind=-1;
	EnvNAVXYTHETALAT3Dpt_t temppose;
	temppose.x = 0.0;
	temppose.y = 0.0;
	temppose.theta = 0.0;
	vector<sbpl_2Dcell_t> footprint;


	numofadditionalzlevs = numofadditionalzlevs_in;
	SBPL_PRINTF("Planning with additional z levels. Number of additional z levels = %d\n", numofadditionalzlevs);

	//allocate memory and set FootprintPolygons for additional levels
	ros::NodeHandle nh("~");
	AddLevelFootprintPolygonV = new vector<sbpl_2Dpt_t> [numofadditionalzlevs];
	for (levelind=0; levelind < numofadditionalzlevs; levelind++)
	{
    char buf[64];
    sprintf(buf,"downprojected_footprint%d",levelind);
    ros::Publisher temp = nh.advertise<geometry_msgs::PolygonStamped>(buf, 10);
    footprintPub.push_back(temp);
		AddLevelFootprintPolygonV[levelind] = perimeterptsV[levelind];
	}

	//compute additional levels action info
	SBPL_PRINTF("pre-computing action data for additional levels:\n");
	AdditionalInfoinActionsV = new EnvNAVXYTHETAMLEVLATAddInfoAction_t*[NAVXYTHETALAT_THETADIRS]; 
	for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, NAVXYTHETALAT_THETADIRS);

		//compute sourcepose
		EnvNAVXYTHETALAT3Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETALAT_THETADIRS);

		AdditionalInfoinActionsV[tind] = new EnvNAVXYTHETAMLEVLATAddInfoAction_t[EnvNAVXYTHETALATCfg.actionwidth]; 

		//iterate over actions for each angle
		for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
		{
			EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[tind][aind];
			
			//initialize delta variables
			AdditionalInfoinActionsV[tind][aind].dX = nav3daction->dX;
			AdditionalInfoinActionsV[tind][aind].dY = nav3daction->dY;
			AdditionalInfoinActionsV[tind][aind].starttheta = tind;
			AdditionalInfoinActionsV[tind][aind].endtheta = nav3daction->endtheta;

			//finally, create the footprint for the action for each level
			AdditionalInfoinActionsV[tind][aind].intersectingcellsV = new vector<sbpl_2Dcell_t>[numofadditionalzlevs];
			for(levelind = 0; levelind < numofadditionalzlevs; levelind++)
			{
				//iterate over the trajectory of the robot executing the action
        set<pair<int,int> > cell_set;
				for (int pind = 0; pind < (int)EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size(); pind++)
				{
				
					//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y)
					EnvNAVXYTHETALAT3Dpt_t pose;
					pose = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[pind];
					pose.x += sourcepose.x;
					pose.y += sourcepose.y;
          printFP = false;//tind==0 && aind==0;
					CalculateFootprintForPose(pose, &cell_set, AddLevelFootprintPolygonV[levelind]);
          printFP = false;

				}
        AdditionalInfoinActionsV[tind][aind].intersectingcellsV[levelind].reserve(cell_set.size());
        for(set<pair<int,int> >::iterator it = cell_set.begin(); it!=cell_set.end(); it++){
          sbpl_2Dcell_t cell;
          cell.x = it->first;
          cell.y = it->second;
          AdditionalInfoinActionsV[tind][aind].intersectingcellsV[levelind].push_back(cell);
        }

				//remove the source footprint
				RemoveSourceFootprint(sourcepose, &AdditionalInfoinActionsV[tind][aind].intersectingcellsV[levelind], AddLevelFootprintPolygonV[levelind]);

			}
		}
	}
  printFootprint("layer0.txt",AdditionalInfoinActionsV[0][0].intersectingcellsV[0]);
  printFootprint("layer1.txt",AdditionalInfoinActionsV[0][0].intersectingcellsV[1]);

	//create maps for additional levels and initialize to zeros (freespace)
	AddLevelGrid2D = new unsigned char** [numofadditionalzlevs];
	for(levelind = 0; levelind < numofadditionalzlevs; levelind++)
	{
		AddLevelGrid2D[levelind] = new unsigned char* [EnvNAVXYTHETALATCfg.EnvWidth_c];
		for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
			AddLevelGrid2D[levelind][xind] = new unsigned char [EnvNAVXYTHETALATCfg.EnvHeight_c];
			for(yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
				AddLevelGrid2D[levelind][xind][yind] = 0;
			}
		}
	}

	//create inscribed and circumscribed cost thresholds
	AddLevel_cost_possibly_circumscribed_thresh = new unsigned char [numofadditionalzlevs];
	AddLevel_cost_inscribed_thresh = new unsigned char [numofadditionalzlevs];
	for(levelind = 0; levelind < numofadditionalzlevs; levelind++)
	{
		AddLevel_cost_possibly_circumscribed_thresh[levelind] = cost_possibly_circumscribed_thresh_in[levelind];
		AddLevel_cost_inscribed_thresh[levelind] = cost_inscribed_thresh_in[levelind];
	}

	return true;
}

bool EnvironmentNAVXYTHETAMLEVLAT3D::setFootprint(int levelind, const vector<sbpl_2Dpt_t> perimeterptsV){
  clock_t t1 = clock();
	EnvNAVXYTHETALAT3Dpt_t temppose;
	temppose.x = 0.0;
	temppose.y = 0.0;
	temppose.theta = 0.0;
	vector<sbpl_2Dcell_t> footprint;

  AddLevelFootprintPolygonV[levelind] = perimeterptsV;

	//compute additional levels action info
	SBPL_PRINTF("pre-computing action data for additional levels:\n");
	for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
	{
    clock_t t0 = clock();
		SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, NAVXYTHETALAT_THETADIRS);

		//compute sourcepose
		EnvNAVXYTHETALAT3Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETALAT_THETADIRS);

		//iterate over actions for each angle
		for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
		{
			//finally, create the footprint for the action for each level
      AdditionalInfoinActionsV[tind][aind].intersectingcellsV[levelind].clear();




      //iterate over the trajectory of the robot executing the action
      set<pair<int,int> > cell_set;
      EnvNAVXYTHETALAT3Dpt_t pose;
      pose = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[0];
      pose.x += sourcepose.x;
      pose.y += sourcepose.y;
      printFP = false;//tind==0 && aind==0;
      CalculateFootprintForPose(pose, &cell_set, AddLevelFootprintPolygonV[levelind]);
      printFP = false;
      vector<pair<double,double> > initial_fp;
      initial_fp.reserve(cell_set.size());
      for(set<pair<int,int> >::iterator it = cell_set.begin(); it!=cell_set.end(); it++)
        initial_fp.push_back(pair<double,double>(DISCXY2CONT(it->first,EnvNAVXYTHETALATCfg.cellsize_m), DISCXY2CONT(it->second,EnvNAVXYTHETALATCfg.cellsize_m)));
      for (int pind = 1; pind < (int)EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size(); pind++)
      {

        //now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y)
        EnvNAVXYTHETALAT3Dpt_t pose;
        pose = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[pind];
        pose.x += sourcepose.x;
        pose.y += sourcepose.y;
        for(int i=0; i<initial_fp.size(); i++){
          pair<int,int> temp_pair;
          temp_pair.first = CONTXY2DISC(cos(pose.theta)*initial_fp[i].first - sin(pose.theta)*initial_fp[i].second + pose.x,EnvNAVXYTHETALATCfg.cellsize_m);
          temp_pair.second = CONTXY2DISC(sin(pose.theta)*initial_fp[i].first + cos(pose.theta)*initial_fp[i].second + pose.y,EnvNAVXYTHETALATCfg.cellsize_m);
          cell_set.insert(temp_pair);
        }
      }




      /*
      set<pair<int,int> > cell_set;
      //iterate over the trajectory of the robot executing the action
      for (int pind = 0; pind < (int)EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size(); pind++)
      {
        //now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y)
        EnvNAVXYTHETALAT3Dpt_t pose;
        pose = EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[pind];
        pose.x += sourcepose.x;
        pose.y += sourcepose.y;
        printFP = false;//tind==0 && aind==0;
				CalculateFootprintForPose(pose, &cell_set, AddLevelFootprintPolygonV[levelind]);
        printFP = false;
      }
      */

      AdditionalInfoinActionsV[tind][aind].intersectingcellsV[levelind].reserve(cell_set.size());
      for(set<pair<int,int> >::iterator it = cell_set.begin(); it!=cell_set.end(); it++){
        sbpl_2Dcell_t cell;
        cell.x = it->first;
        cell.y = it->second;
        AdditionalInfoinActionsV[tind][aind].intersectingcellsV[levelind].push_back(cell);
      }

      //remove the source footprint
      RemoveSourceFootprint(sourcepose, &AdditionalInfoinActionsV[tind][aind].intersectingcellsV[levelind], AddLevelFootprintPolygonV[levelind]);
		}
    printf("angle %d took %f\n",tind,double(clock()-t0)/CLOCKS_PER_SEC);
	}
  printf("footprint computation took %f\n",double(clock()-t1)/CLOCKS_PER_SEC);
  printFootprint("layer0.txt",AdditionalInfoinActionsV[0][3].intersectingcellsV[0]);
  printFootprint("layer1.txt",AdditionalInfoinActionsV[0][0].intersectingcellsV[1]);

	return true;
}

//set 2D map for the additional level levind
bool EnvironmentNAVXYTHETAMLEVLAT3D::Set2DMapforAddLev(const unsigned char* mapdata, int levind)
{
	int xind=-1, yind=-1;

	if(AddLevelGrid2D == NULL)
	{
		SBPL_ERROR("ERROR: failed to set2Dmap because the map was not allocated previously\n");
		return false;
	}

	for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
		for(yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
			AddLevelGrid2D[levind][xind][yind] = mapdata[xind+yind*EnvNAVXYTHETALATCfg.EnvWidth_c];
		}
	}
	
	return true;
}

//set 2D map for the additional level levind
//the version of Set2DMapforAddLev that takes newmap as 2D array instead of one linear array
bool EnvironmentNAVXYTHETAMLEVLAT3D::Set2DMapforAddLev(const unsigned char** NewGrid2D, int levind)
{
	int xind=-1, yind=-1;

	if(AddLevelGrid2D == NULL)
	{
		SBPL_ERROR("ERROR: failed to set2Dmap because the map was not allocated previously\n");
		return false;
	}

	for (xind = 0; xind < EnvNAVXYTHETALATCfg.EnvWidth_c; xind++) {
		for(yind = 0; yind < EnvNAVXYTHETALATCfg.EnvHeight_c; yind++) {
			AddLevelGrid2D[levind][xind][yind] = NewGrid2D[xind][yind];
		}
	}
	
	return true;
}



  /*
	update the traversability of a cell<x,y> in addtional level zlev (this is not to update basic level)
  */
bool EnvironmentNAVXYTHETAMLEVLAT3D::UpdateCostinAddLev(int x, int y, unsigned char newcost, int zlev)
  {

#if DEBUG
	//SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d at level %d from old cost=%d to new cost=%d\n",x,y,zlev,AddLevelGrid2D[zlev][x][y],newcost);
#endif

	AddLevelGrid2D[zlev][x][y] = newcost;

	//no need to update heuristics because at this point it is computed solely based on the basic level

	return true;
  }

void EnvironmentNAVXYTHETAMLEVLAT3D::visualizeFootprints(){
	geometry_msgs::PolygonStamped footprint_poly;
	footprint_poly.header.frame_id = "base_footprint";
	footprint_poly.header.stamp = ros::Time::now();

	footprint_poly.polygon.points.resize(EnvNAVXYTHETALATCfg.FootprintPolygon.size());
	for(unsigned int i = 0; i < EnvNAVXYTHETALATCfg.FootprintPolygon.size(); ++i){
    footprint_poly.polygon.points[i].x = EnvNAVXYTHETALATCfg.FootprintPolygon[i].x;
    footprint_poly.polygon.points[i].y = EnvNAVXYTHETALATCfg.FootprintPolygon[i].y;
    footprint_poly.polygon.points[i].z = 0.0;
	}
	m_footprintPolygonPub.publish(footprint_poly);

  for(int i=0; i<numofadditionalzlevs; i++){
    footprint_poly.polygon.points.resize(AddLevelFootprintPolygonV[i].size());
    for(unsigned int j=0; j<AddLevelFootprintPolygonV[i].size(); ++j){
      footprint_poly.polygon.points[j].x = AddLevelFootprintPolygonV[i][j].x;
      footprint_poly.polygon.points[j].y = AddLevelFootprintPolygonV[i][j].y;
      if(i==0)
        footprint_poly.polygon.points[j].z = 1.213;
      else if(i==1)
        footprint_poly.polygon.points[j].z = 0.450;
    }
    footprintPub[i].publish(footprint_poly);
  }
  //base 0.0 to 0.3
  //spine 0.25 to 1.4
  //arm 0.65 to 0.95
}

//------------------------------------------------------------------------------

void EnvironmentNAVXYTHETAMLEVLAT3D::drawStuff(EnvNAVXYTHETALATAction_t* action, int x, int y){
  vector<sbpl_2Dcell_t>* intersectingcellsV = &AdditionalInfoinActionsV[(unsigned int)action->starttheta][action->aind].intersectingcellsV[0]; 
  sbpl_2Dcell_t cell;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_projected->header.frame_id = "map";
  cloud_projected->header.stamp = ros::Time(0);
  pcl::PointXYZ pt;
  for(unsigned int i = 0; i < (int)intersectingcellsV->size(); i++){
    cell = intersectingcellsV->at(i);
    pt.x = DISCXY2CONT(cell.x,EnvNAVXYTHETALATCfg.cellsize_m) + DISCXY2CONT(x,EnvNAVXYTHETALATCfg.cellsize_m) - 12.487500;
    pt.y = DISCXY2CONT(cell.y,EnvNAVXYTHETALATCfg.cellsize_m) + DISCXY2CONT(y,EnvNAVXYTHETALATCfg.cellsize_m) - 12.487500;
    pt.z = 1.12;
    cloud_projected->points.push_back(pt);
  }
  footPointsPub.publish(cloud_projected);

  FILE* fout = fopen("crap_map.csv","w");
  for(int ix=0; ix<EnvNAVXYTHETALATCfg.EnvWidth_c; ix++){
    for(int iy=0; iy<EnvNAVXYTHETALATCfg.EnvHeight_c; iy++)
      fprintf(fout,"%d ",AddLevelGrid2D[0][ix][iy]);
    fprintf(fout,"\n");
  }
  fclose(fout);
  sleep(2.0);
}

