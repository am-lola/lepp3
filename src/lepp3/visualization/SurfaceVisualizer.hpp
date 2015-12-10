#ifndef LEPP2_VISUALIZATION_SURFACE_VISUALIZER_H__
#define LEPP2_VISUALIZATION_SURFACE_VISUALIZER_H__

#include <vector>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "lepp3/VideoObserver.hpp"
#include "lepp3/SurfaceAggregator.hpp"

namespace lepp {

template<class PointT>
class SurfaceVisualizer: public VideoObserver<PointT>, public SurfaceAggregator<
		PointT> {
public:
	// EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SurfaceVisualizer() :
            viewer_("SurfaceVisualizer") {
	} //pclViz (new pcl::visualization::PCLVisualizer ("PCL_VIZ"))

	/**
	 * VideoObserver interface implementation: show the current point cloud.
	 */
	virtual void notifyNewFrame(int idx,
			const typename pcl::PointCloud<PointT>::ConstPtr& pointCloud) {
		viewer_.showCloud(pointCloud);
	}

	/**
     * SurfaceAggregator interface implementation: processes detected surfaces.
	 */
    virtual void updateSurfaces(
            std::vector<typename pcl::PointCloud<PointT>::ConstPtr> cloud_surfaces);

    void drawSurfaces(
            std::vector<typename pcl::PointCloud<PointT>::ConstPtr> surfaces,
			pcl::visualization::PCLVisualizer& viewer);

private:
	/**
	 * Used for the visualization of the scene.
	 */
	pcl::visualization::CloudViewer viewer_;

	// boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViz;
};

template<class PointT>
void SurfaceVisualizer<PointT>::drawSurfaces(
        std::vector<typename pcl::PointCloud<PointT>::ConstPtr> surfaces,
		pcl::visualization::PCLVisualizer& pclViz) {

     pclViz.removePointCloud("SUR1",0);
     pclViz.removePointCloud("SUR2",0);
     pclViz.removePointCloud("SUR3",0);
     pclViz.removePointCloud("SUR4",0);

    size_t const sz = surfaces.size();
	for (size_t i = 0; i < sz; ++i) {

		 if (i ==0) {
            if (!pclViz.updatePointCloud(surfaces[i], "SUR1"))
                pclViz.addPointCloud(surfaces[i], "SUR1");                       //RED
			pclViz.setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0, 0,
                    "SUR1");
		}
		else if (i == 1) {
            if (!pclViz.updatePointCloud(surfaces[i], "SUR2"))                  //GREEN
                pclViz.addPointCloud(surfaces[i], "SUR2");
			pclViz.setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1.0, 0,
                    "SUR2");
		}
		else if (i == 2) {
            if (!pclViz.updatePointCloud(surfaces[i], "SUR3"))                  //BLUE
                pclViz.addPointCloud(surfaces[i], "SUR3");
			pclViz.setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1.0,
                    "SUR3");
		}
		else if (i == 3) {
            if (!pclViz.updatePointCloud(surfaces[i], "SUR4"))
                pclViz.addPointCloud(surfaces[i], "SUR4");                       //PURPLE
			pclViz.setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 0.580392, 0,
                    0.827451, "SUR4");
		}
//		else if (i == 5) {
//			if (!pclViz.updatePointCloud(surfaces[i], "SUR5"))
//				pclViz.addPointCloud(surfaces[i], "SUR5");                      //YELLOW
//			pclViz.setPointCloudRenderingProperties(
//					pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.843137, 0,
//					"SUR5");
//		}
	}

	std::cout << "exiting stair drawer..." << std::endl;
	std::cout << "=======================" << std::endl;
}

template<class PointT>
void SurfaceVisualizer<PointT>::updateSurfaces(
        std::vector<typename pcl::PointCloud<PointT>::ConstPtr> surfaces) {
    std::cout << "entered updateSurfaces" << std::endl;
    pcl::visualization::CloudViewer::VizCallable surface_visualization =
            boost::bind(&SurfaceVisualizer::drawSurfaces, this, surfaces, _1);
	std::cout << "call runOnVisualizationThreadOnce" << std::endl;
    viewer_.runOnVisualizationThread(surface_visualization);
    std::cout << "exiting updateSurfaces..." << std::endl;
}

} // namespace lepp
#endif
