/*
 * Azure Kinect Camera Class Header File
 *
 * Xiao Song ( xiaosx@berkeley.edu )
 * 
 */

#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp> // cv::Size
#include <Eigen/Core> // Eigen::aligned_allocator

#include "CameraSetup.h"
#include "Types.h" // CameraParameter, MultiCameraFrame, ImuPair, CameraParameter

namespace ark
{

    class AzureKinectCamera : public CameraSetup
    {
    public:
        AzureKinectCamera();

        AzureKinectCamera(const CameraParameter&);

        ~AzureKinectCamera();

        /*
         *
         *
         * 
         * 
         */
        void start() override;
        
        /*
         * Get latest datas (frames) from sensor stream. 
         * And store TODO
         * 
         * 
         */
        void update(MultiCameraFrame::Ptr) override;

        std::string getModelName() const override;

        /*
         * Intrinsic matrix of RGB Camera
         */
        std::vector<float> getColorIntrinsics() override;

        /*
         *
         *
         */
        cv::Size getImageSize() const override;

        double getDepthScale();

        /*
         * Get IMU data based on time stamp
         *
         * Eigen::aligned_allocator is used to align data that contain Eigen component
         *  this related to how C++98 - C++14 work with dynamic memory allocation.
         *  https://eigen.tuxfamily.org/dox/classEigen_1_1aligned__allocator.html
         *  https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html 
         *  https://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
         */
        bool getImuToTime(double, std::vector<ImuPair, Eigen::aligned_allocator<ImuPair>>&);

    protected:
        CameraParameter camParam;
    };

}