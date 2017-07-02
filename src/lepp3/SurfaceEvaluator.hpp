//
// Created by fabian on 03.03.17.
//
#ifndef LEPP3_SURFACEEVALUATOR_HPP
#define LEPP3_SURFACEEVALUATOR_HPP
#include <boost/filesystem.hpp>
#include <iostream>
#include <sstream>
#include "lepp3/FrameData.hpp"
#include "lepp3/util/util.h"
class SurfaceEvaluator: public FrameDataObserver {
public:
    SurfaceEvaluator();
    /**
   * SurfaceAggregator interface implementation: processes the current models.
   */
    void updateFrame(FrameDataPtr frameData);
private:
    void init();
    float evaluate(SurfaceModelPtr const& model);
    double getAngle(const pcl::ModelCoefficients &coeffs);
    std::string file_path_;
    std::vector<SurfaceModelPtr> surfaces;
    std::vector<size_t> indeces;
};
SurfaceEvaluator::SurfaceEvaluator() {
    init();
}
float SurfaceEvaluator::evaluate(SurfaceModelPtr const &model) {
    //model->get_hull()->points[0].x
    float tmp = 0;
    for(int i = 0; i < model->get_hull()->points.size(); ++i) {
        if (i != model->get_hull()->points.size() - 1) {
            float mul_a = model->get_hull()->points[i].x * model->get_hull()->points[i+1].y;
            float mul_b = model->get_hull()->points[i+1].x * model->get_hull()->points[i].y;
            tmp = tmp + (mul_a - mul_b);
        } else {
            float mul_a = model->get_hull()->points[i].x * model->get_hull()->points[0].y;
            float mul_b = model->get_hull()->points[0].x * model->get_hull()->points[i].y;
            tmp = tmp + (mul_a - mul_b);
        }
    }
    tmp *= 0.5;
    float area = std::abs(tmp);
//    std::stringstream ss;
//    ss << model->id() << ","
//       << area << ","
//       << std::endl;
//    std::ofstream tf_fout;
//    // open the file and add the current model evaluation to the end of it
//    tf_fout.open(file_path_.c_str(), std::ofstream::app);
//    tf_fout << ss.str();
//    tf_fout.close();
    return area;
}
double SurfaceEvaluator::getAngle(const pcl::ModelCoefficients &coeffs) {
    float dot_product = (coeffs.values[0] * 0) + (coeffs.values[1] * 0) + (coeffs.values[2] * 1);
    float coeff_length = std::cbrt(coeffs.values[0] * coeffs.values[0] + coeffs.values[1] * coeffs.values[1] + coeffs.values[2] * coeffs.values[2]);
    double angle = acos(dot_product/coeff_length) * 180 / M_PI;
    return angle;
}
void SurfaceEvaluator::init() {
    namespace bfs = boost::filesystem;
    std::stringstream ss;
    // Create the evaluation directory
    ss << "../surfaceEvaluation/";
    std::string dir = ss.str();
    if ( !bfs::exists(bfs::path(dir)) )
        bfs::create_directory(bfs::path(dir));
    // Create the sub directory based on current timestamp
    ss << lepp::get_current_timestamp();
    dir = ss.str();
    bfs::create_directory(bfs::path(dir));
    // Prepare the csv file path
    ss  << "/surfeval.csv";
    file_path_ = ss.str();
    std::cout << "file_path: " << file_path_ << std::endl;
    // Create the file header
    std::ofstream tf_fout;
    tf_fout.open(file_path_.c_str(), std::ofstream::app);
    tf_fout << "model id,"
            << "est. surface,"
            << "total surface,"
            << "angle,"
            << std::endl;
    tf_fout.close();
}
void SurfaceEvaluator::updateFrame(FrameDataPtr frameData) {
    size_t sz = frameData->surfaces.size();
    cout << sz << endl;
    if (sz > 0) {
        float total_area = 0;
        /* Use this implementation if point of view creates multiple surfaces for one object
        if (sz > 2) {
            for (size_t i = 0; i < sz; ++i) {
                SurfaceModelPtr current = frameData->surfaces[i];
                float ref_x = current->get_planeCoefficients().values[0];
                float ref_y = current->get_planeCoefficients().values[1];
                float ref_z = current->get_planeCoefficients().values[2];
                bool surface_flag = false;
                for (size_t j = i+1; j < sz; ++j) {
                    SurfaceModelPtr comp = frameData->surfaces[j];
                    float comp_x = comp->get_planeCoefficients().values[0];
                    float comp_y = comp->get_planeCoefficients().values[1];
                    float comp_z = comp->get_planeCoefficients().values[2];
                    if (ref_x == comp_x && ref_y == comp_y && ref_z == comp_z) {
                        surface_flag = true;
                        indeces.push_back(j);
                    }
                }
                if (surface_flag) {
                    indeces.push_back(i);
                }
            }
            for (size_t i = 0; i < sz; ++i) {
                bool contain = true;
                for (size_t j = 0; j < indeces.size(); ++j) {
                    if (indeces[j] == i) {
                        contain = false;
                    }
                }
                if (contain) {
                    surfaces.push_back(frameData->surfaces[i]);
                }
            }
            if (surfaces.size() == 2) {
                double ref_z = surfaces[0]->centerpoint().z;
                double comp_z = surfaces[1]->centerpoint().z;
                if (ref_z > comp_z)
                    surfaces.erase(surfaces.begin() + 1);
                else
                    surfaces.erase(surfaces.begin());
            }
            for (size_t i = 0; i < surfaces.size(); ++i) {
            float area = evaluate(surfaces[i]);
            total_area += area;
            std::stringstream ss;
            ss << surfaces[i]->id() << ","
               << area << ","
               << total_area
               << std::endl;
            std::ofstream tf_fout;
            // open the file and add the current model evaluation to the end of it
            tf_fout.open(file_path_.c_str(), std::ofstream::app);
            tf_fout << ss.str();
            tf_fout.close();
            }
            surfaces.clear();
        } else {
            float area = evaluate(frameData->surfaces[0]);
            total_area += area;
            std::stringstream ss;
            ss << frameData->surfaces[0]->id() << ","
               << area << ","
               << total_area
               << std::endl;
            std::ofstream tf_fout;
            // open the file and add the current model evaluation to the end of it
            tf_fout.open(file_path_.c_str(), std::ofstream::app);
            tf_fout << ss.str();
            tf_fout.close();
        }
         */
        for (size_t i = 0; i < sz; ++i) {
            double angle = getAngle(frameData->surfaces[i]->get_planeCoefficients());
            float area = evaluate(frameData->surfaces[i]);
            total_area += area;
            std::stringstream ss;
            ss << frameData->surfaces[i]->id() << ","
               << area << ","
               << total_area << ","
               << angle << std::endl;
            std::ofstream tf_fout;
            tf_fout.open(file_path_.c_str(), std::ofstream::app);
            tf_fout << ss.str();
            tf_fout.close();
        }
    }
}
#endif //LEPP3_SURFACEEVALUATOR_HPP