/* 
 * Azure Kinect Camera Class Implementation for Hand & Avatr
 * 
 * Alex Yu ( alexyu99126@gmail.com ) 2019
 * Xiao Song ( xiaosx@berkeley.edu ) 2021
 * 
 * Change Log
 * 2021-10-25: optimize the implementation.
 */

#include <cstring> // memcpy
#include <k4a/k4a.h>
#include <iostream>
#include "openark/stdafx.h"
#include "openark/Version.h"
#include "openark/camera/AzureKinectCamera.h"

/** Azure Kinect Cross-Platform Depth Camera Backend **/
namespace ark 
{
    // TODO add resource release
    AzureKinectCamera::AzureKinectCamera() noexcept : \
        device(NULL), capture(NULL), \
        calibration(NULL), transformation(NULL) \
    {
        // Count number of k4a device to ensure there are connected device
        if ( k4a_device_get_installed_count() == 0 )
        {
            std::cerr << "Fatal: No Azure Kinect (K4A) devices found" << std::endl ;
            badInputFlag = true;
            return;
        }

        // Ensure camera can be open properly
        if ( K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
        {
            std::cerr << "Fatal: Failed to open Azure Kinect (K4A) device" << std::endl ;
            badInputFlag = true;
            return;
        }

        /* Camera configuration setting */
        camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        // currently only fixed frame rate is supported
        camera_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        // The Azure Kinect device does not natively capture in this format. 
        // Requesting images of this format requires additional computation in the API.
        camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; 
        // 1280 * 720  16:9
        camera_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        // 1024 * 1024 wide field of view depth without binned
        camera_config.depth_mode       = K4A_DEPTH_MODE_WFOV_UNBINNED;
        // ensures that depth and color images are both available in the capture
        // objects may be produced only a single image when the corresponding image is dropped.
        camera_config.synchronized_images_only = true; 

        /* Get calibration information */
        if (K4A_RESULT_SUCCEEDED != \
            k4a_device_get_calibration(device, \
                                       camera_config.depth_mode, \
                                       camera_config.color_resolution, \
                                       &calibration))
        {
            std::cerr << "Fatal: Failed to get Azure Kinect (K4A) device calibration" << std::endl;;
            badInputFlag = true;
            return;
        }

        /* Set the intrinsics */
        auto& color_camera_intrin = calibration.color_camera_calibration.intrinsics.parameters.param;
        intrinsic.at(0) = color_camera_intrin.cx;
        intrinsic.at(1) = color_camera_intrin.cy;
        intrinsic.at(2) = color_camera_intrin.fx;
        intrinsic.at(3) = color_camera_intrin.fy;

        img_width = calibration.color_camera_calibration.resolution_width;
        img_height = calibration.color_camera_calibration.resolution_height;

        // Create depth/coor transformation
        transformation = k4a_transformation_create(&calibration);

        // Start camera
        if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &camera_config))
        {
            std::cerr << "Fatal: Failed to start Azure Kinect camera" << std::endl;;
            badInputFlag = true;
            return;
        }
    }

    // TODO
    AzureKinectCamera::~AzureKinectCamera() 
    {
        auto device = reinterpret_cast<k4a_device_t>(this->k4a_device);
        if (device != nullptr) {
            k4a_device_close(device);
        }
        auto transformation = reinterpret_cast<k4a_transformation_t>(this->k4a_transformation);
        if (transformation != nullptr) {
            k4a_transformation_destroy(transformation);
        }
        auto xy_table = reinterpret_cast<k4a_image_t>(this->xy_table_cache);
        if (xy_table != nullptr) {
            k4a_image_release(xy_table);
        }
    }

    const std::string AzureKinectCamera::getModelName() const 
    {
        return std::string{"Azure Kinect"};
    }

    int AzureKinectCamera::getWidth() const 
    {
		return scaled_width;
    }

    int AzureKinectCamera::getHeight() const 
    {
		return scaled_height;
    }

    // TODO
	const DetectionParams::Ptr & AzureKinectCamera::getDefaultParams() const 
    {
		if (!defaultParamsSet) {
			defaultParamsSet = true;
			defaultParams = std::make_shared<DetectionParams>();
			defaultParams->contourImageErodeAmount = 0;
			defaultParams->contourImageDilateAmount = 2;
			defaultParams->fingerCurveFarMin = 0.18;
			defaultParams->fingerLenMin = 0.025;
			defaultParams->handClusterInterval = 15;
			defaultParams->handClusterMaxDistance = 0.003;
			defaultParams->handSVMConfidenceThresh = 0.52;
			defaultParams->handClusterMinPoints = 0.015;
			defaultParams->planeFloodFillThreshold = 0.19;
			defaultParams->planeEquationMinInliers = 0.02;
			defaultParams->planeMinPoints = 0.02;
			defaultParams->planeCombineThreshold = 0.0019;
			defaultParams->normalResolution = 3;
			defaultParams->handRequireEdgeConnected = false;
		}
		return defaultParams;
    }

	bool AzureKinectCamera::hasRGBMap() const 
    {
        return true;
	}

    // tODO question
    uint64_t AzureKinectCamera::getTimestamp() const
    {
        return timestamp;
    }

    cv::Vec4d AzureKinectCamera::getCalibIntrinsics() const
    {
        return intrinsic;
    }

    // TODO add resource release
    // TODO not sure about resource release
    void AzureKinectCamera::update(cv::Mat & xyz_output, \
                                   cv::Mat & rgb_output, \
                                   cv::Mat & ir_map, \
                                   cv::Mat & amp_map, \
                                   cv::Mat & flag_map )
    
    {
        // Runtime resource
        k4a_image_t image_sample = NULL;
        k4a_image_t depth_sample = NULL;
        k4a_image_t depth_sample_transformed_img_coord = NULL;
        k4a_image_t depth_sample_transformed_point_cloud = NULL;
    
        // Capture depth & image data
        capture = NULL;
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                std::cerr << "Warning: Timed out waiting for a capture from Azure Kinect" << std::endl;;
                badInputFlag = true;
                goto ReleaseRunTimeResource;
            case K4A_WAIT_RESULT_FAILED:
                std::cerr << "Warning: Failed to read a capture from Azure Kinect" << std::endl;;
                badInputFlag = true;
                goto ReleaseRunTimeResource;
        }

        // Extract depth from capture
        depth_image = k4a_capture_get_depth_image(capture);
        if ( !depth_image ) 
        {
            std::cerr << "Warning: Failed to get depth image from Azure Kinect capture" << std::endl;;
            xyz_output = xyzMap.clone();
            rgb_output = rgbMap.clone();
            goto ReleaseRunTimeResource;
        }

        // Extract BGRA/GRAY from capture
        image_sample = k4a_capture_get_image_sample(capture);
        if (image_sample == NULL) 
        {
            std::cerr << "Warning: Failed to get color image from Azure Kinect capture" << std::endl;;
            xyz_output = xyzMap.clone();
            rgb_output = rgbMap.clone();
            goto ReleaseRunTimeResource;
        }
        badInputFlag = false;

        // Extract device (the camera) time stamp & convert to nanoseconds
        timestamp = k4a_image_get_timestamp_usec(depth_image) * 1e3;
    
        // Copy result to output
        // Create a cv:::Mat using the k4a_image_t image buffer, there is no memory copy in this step
        // https://stackoverflow.com/questions/57222190/how-to-convert-k4a-image-t-to-opencv-matrix-azure-kinect-sensor-sdk
        cv::Mat rgba_output(img_height, \
                            img_width, \
                            CV_8UC4, \
                            reinterpret_cast<void*>k4a_image_get_buffer(image_sample), \
                            cv::Mat::AUTO_STEP);

        // Convert RGBA to BGR using opencv
        // openCV use vectorize implementation with -O2 -O3. So it's faster then element wise operation
        // User may reuse the rgb_output variable to avoid allocate new memory every time
        cv::cvtColor(rgba_output, rgb_output, cv::COLOR_RGBA2RGB);

        // Align depth to color image
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                    img_width,
                                                    img_height,
                                                    img_width * (int)sizeof(uint16_t), 
                                                    &depth_sample_transformed_img_coord))
        {
            std::cerr << "Failed to create transformed depth image" << std::endl;
            goto ReleaseRunTimeResource;
        }

        // TODO change this
        if ( K4A_RESULT_SUCCEEDED != \
             k4a_image_create_from_buffer( K4A_IMAGE_FORMAT_CUSTOM,
                                           img_width,
                                           img_height,
                                           img_width * 3 * (int)sizeof(int16_t),
                                           reinterpret_cast<uint8_t*> xyz_output.data,
                                           img_width * img_width * 3 * (int)sizeof(int16_t),
                                           nullptr,
                                           nullptr,
                                           point_cloud_transformed_image_coordinate ))
        {
            printf("Failed to create point cloud buffer from cv::Mat memory\n");
            goto ReleaseRunTimeResource;
        }

        // Create XYZ map from transformed depth
        // K4A_CALIBRATION_TYPE_COLOR is used because depth is reproject to camera coordinate first
        // Each pixel of the xyz_image (point_cloud_transformed_image_coordinate) consists of three int16_t values
        //
        // Azure-Kinect-Sensor-SDK/examples/transformation/main.cpp
        // https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/group___functions_ga7385eb4beb9d8892e8a88cf4feb3be70.html
        if (K4A_RESULT_SUCCEEDED != \
            k4a_transformation_depth_image_to_point_cloud(transformation,
                                                          depth_sample_transformed_img_coord,
                                                          K4A_CALIBRATION_TYPE_COLOR,
                                                          depth_sample_transformed_point_cloud))
        {
            printf("Failed to compute point cloud\n");
            goto ReleaseRunTimeResource;
        }

        // TODO add convert point cloud to cv mat

    ReleaseRunTimeResource:
        if ( image_sample ) k4a_image_release( image_sample );
        if ( depth_sample ) k4a_image_release( depth_sample );
        if ( depth_sample_transformed_img_coord ) k4a_image_release( depth_sample_transformed_img_coord );
        if ( depth_sample_transformed_point_cloud ) k4a_image_release( depth_sample_transformed_point_cloud );
        k4a_capture_release(capture);
    }
}
