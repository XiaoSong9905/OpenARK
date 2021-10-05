#include "D435iCamera.h"
#include "OkvisSLAMSystem.h"
#include <iostream>
#include <thread>
#include "glfwManager.h"
#include "Util.h"

using namespace ark;

int main(int argc, char **argv)
{
    if (!MyGUI::Manager::init())
    {
        fprintf(stdout, "Failed to initialize GLFW\n");
        return -1;
    }
    std::string configFilename;
    if (argc > 1) configFilename = argv[1];
    else configFilename = util::resolveRootPath("config/d435i_intr.yaml");

    cv::FileStorage configFile(configFilename, cv::FileStorage::READ);

    CameraParameter cameraParameter;
    if (configFile["emitterPower"].isReal()) {
        configFile["emitterPower"] >> cameraParameter.emitterPower;
    }
    D435iCamera camera(cameraParameter);
    camera.start();
    MyGUI::ImageWindow test_camera_win("Test Camera Viewer", 300, 300,  GL_RGB, GL_UNSIGNED_BYTE);
    while (MyGUI::Manager::running())
    {
        //Update the display
        MyGUI::Manager::update();
        try
        {
            //Get current camera frame
            MultiCameraFrame::Ptr frame = std::allocate_shared<MultiCameraFrame>(Eigen::aligned_allocator<MultiCameraFrame>());
            camera.update(frame);
            test_camera_win.set_image(frame->images_[3]);
        }
        catch (const std::exception &e)
        {
            std::cout << "const std::exception &e\n";
            std::cerr << e.what() << '\n'; 
        }
        catch (...)
        {
            std::cout << "An exception caught.\n";
        }
    }

    return 0;
}
