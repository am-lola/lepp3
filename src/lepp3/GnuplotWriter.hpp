#ifndef LEPP3_GNUPLOT_WRITER__
#define LEPP3_GNUPLOT_WRITER__

#include "lepp3/Typedefs.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

namespace lepp {


struct GnuplotWriter
{
	/**
	* Writes out 3 files with data points of OldHull, NewHull, and MergeHull respectively.
	*
	* To display plots, navigate to folder and execute the following command to see the third detected surface of the 10th frame.
	* gnuplot -e "splot 'OldHull_10_2.dat' with lines, 'NewHull_10_2.dat' with lines, 'MergeHull_10_2.dat' with lines; pause -1"
	*/
	static void writeHulls(long frameNum, int surfaceNum, PointCloudConstPtr oldHull, 
		PointCloudConstPtr newHull, PointCloudConstPtr mergeHull)
	{
		if (oldHull->size() == 0)
			return;

		std::stringstream sstrm;
		
		ofstream hullWriter;
		sstrm << "./build/GnuPlots/OldHull_" << frameNum << "_" << surfaceNum << ".dat";
		hullWriter.open (sstrm.str().c_str());
		for (size_t i = 0; i < oldHull->size()+1; i++)
		{
			PointT p = oldHull->at(i % oldHull->size());
			hullWriter << p.x << " " << p.y << " " << p.z << endl;
		}
		hullWriter.close();

		sstrm.str(std::string());
		sstrm << "./build/GnuPlots/NewHull_" << frameNum << "_" << surfaceNum << ".dat";
		hullWriter.open (sstrm.str().c_str());
		for (size_t i = 0; i < newHull->size()+1; i++)
		{
			PointT p = newHull->at(i % newHull->size());
			hullWriter << p.x << " " << p.y << " " << p.z << endl;
		}
		hullWriter.close();

		sstrm.str(std::string());
		sstrm << "./build/GnuPlots/MergeHull_" << frameNum << "_" << surfaceNum << ".dat";
		hullWriter.open (sstrm.str().c_str());
		for (size_t i = 0; i < mergeHull->size()+1; i++)
		{
			PointT p = mergeHull->at(i % mergeHull->size());
			hullWriter << p.x << " " << p.y << " " << p.z << endl;
		}
		hullWriter.close();
	}

};


}// namespace lepp
#endif