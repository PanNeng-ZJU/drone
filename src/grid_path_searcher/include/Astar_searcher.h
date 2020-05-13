#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <utility>
#include "backward.hpp"
#include "node.h"
#include <visualization_msgs/Marker.h>
//#include <waypoint_trajectory_generator/Trajectoy.h>
//#include <waypoint_trajectory_generator/trajpoint.h>


class AstarPathFinder
{	
	private:

	protected:
		uint8_t * data;

		uint8_t * data_high_resolution;
		int resolution_ratio=1;//4

		GridNodePtr *** GridNodeMap;
		Eigen::Vector3i goalIdx;
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;

		double resolution, inv_resolution;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;

		GridNodePtr terminatePtr;
		std::multimap<double, GridNodePtr> openSet;

		double getHeu(GridNodePtr node1, GridNodePtr node2);
        double getG(GridNodePtr node1, GridNodePtr node2);

		void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		

    	bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Eigen::Vector3i & index) const;
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Eigen::Vector3i & index) const;

        bool Nodes_if_in_Path(std::vector<Eigen::Vector3d> path,Eigen::Vector3d node);
		
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

	public:
		AstarPathFinder(){};
		~AstarPathFinder(){};
		void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		void resetGrid(GridNodePtr ptr);
		void resetUsedGrids();

		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		void setObs(const double coord_x, const double coord_y, const double coord_z);

		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
		std::vector<Eigen::Vector3d> getPath();
		std::vector<Eigen::Vector3d> getVisitedNodes();
		std::vector<Eigen::Vector3d> getTurningPoints();
		// std::pair<std::vector<Eigen::Vector3d>,nav_msgs::Path>  getSimplifiedPoints();
		std::vector<Eigen::Vector3d> getSimplifiedPoints(int max_gap);
		std::vector<Eigen::Vector3d> getSimplifiedPoints_by_lines();
		std::vector<Eigen::Vector3d> pathSimplify(const std::vector<Eigen::Vector3d> &path, const double path_resolution);
		// void getTurningPoints();
		nav_msgs::Path vector3d_to_waypoints(std::vector<Eigen::Vector3d> path);
		bool if_collision(int x,int y,int z);
};

#endif