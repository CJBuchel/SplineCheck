#include "main.h"

#include <iostream>

int main() {

	// Hermite trajectory
	wayfinder::Path<wayfinder::Hermite::Spline2D> pathHermite;
	pathHermite.setTrajectory({{
		{ // 0
			{0,0}, // p1
			{1,1}, // p2
			{2,2}, // t1
			{3,3} // t2
		},
		
		{ // 1
			{4,4}, // p1
			{5,5}, // p2
			{6,6}, // t1
			{7,7} // t2
		}
	}});

	std::cout << "Path Hermite output: " << pathHermite.get()->waypoints.points[1].t2.y << std::endl;


	// Catmull-Rom trajectory
	wayfinder::Path<wayfinder::CatmullRom::Spline2D> pathCatmullRom;
	pathCatmullRom.setTrajectory({{
		{0,0}, // p1
		{2,0}, // p2
		{0,0}, // p3
		{0,0} // p4
	}});

	std::cout << "Catmull-Rom output: " << pathCatmullRom.get()->waypoints.points[1].x << std::endl;

	// Linear trajectory
	wayfinder::Path<wayfinder::Linear::Line2D> pathLinear;
	pathLinear.setTrajectory({{
		{0,0}, // p1
		{1,1} // p2
	}});

	std::cout << "Path Linear output: " << pathLinear.get()->waypoints.points[1].x << std::endl;

	std::cout << "Program Finished" << std::endl;
	return 0;
}