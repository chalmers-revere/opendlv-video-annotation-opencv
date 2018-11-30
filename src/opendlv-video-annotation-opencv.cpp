/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

std::mutex g_dataMutex{};
bool g_updateCoords{false};
uint16_t g_x{0};
uint16_t g_y{0};

static void onMouse(int event, int x, int y, int, void*) {
    std::lock_guard<std::mutex> lck(g_dataMutex);
    if (event == cv::EVENT_LBUTTONDOWN) {
      g_updateCoords = true;
    }
    if (event == cv::EVENT_LBUTTONUP) {
      g_updateCoords = false;
    }

    if (g_updateCoords) {
      g_x = x;
      g_y = y;
    }
}

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area>" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img.argb --width=640 --height=480" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            cv::namedWindow(sharedMemory->name().c_str(), 0);
            cv::resizeWindow(sharedMemory->name().c_str(), WIDTH, HEIGHT);
            cv::waitKey(10);
            cv::setMouseCallback(sharedMemory->name().c_str(), onMouse, 0);

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Handler to receive distance readings (realized as C++ lambda).
            std::mutex lastSampleTimeStampMutex{};
            cluon::data::TimeStamp lastSampleTimeStamp{};
            auto onNewImage = [&lastSampleTimeStampMutex, &lastSampleTimeStamp](cluon::data::Envelope &&env){
                std::lock_guard<std::mutex> lck(lastSampleTimeStampMutex);
                lastSampleTimeStamp = env.sampleTimeStamp();
            };

            // Register lambda to handle incoming frames.
            od4.dataTrigger(opendlv::proxy::ImageReading::ID(), onNewImage);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                // Display image.
                cv::imshow(sharedMemory->name().c_str(), img);
                cv::waitKey(1);
                {
                    bool send{false};
                    opendlv::logic::sensation::Direction d;

                    {
                      std::lock_guard<std::mutex> lck(g_dataMutex);
                      if (g_updateCoords) {
                          send = true;
                          std::cout << "(" << g_x << ";" << g_y << ")" << std::endl;
                          d.azimuthAngle(g_x).zenithAngle(g_y);
                      }
                    }
                    if (send) {
                        std::lock_guard<std::mutex> lck(lastSampleTimeStampMutex);
                        if (lastSampleTimeStamp.microseconds() == 0) {
                            lastSampleTimeStamp.seconds(lastSampleTimeStamp.seconds()-1).microseconds(999999);
                        }
                        else {
                            lastSampleTimeStamp.microseconds(lastSampleTimeStamp.microseconds()-1);
                        }
                        od4.send(d, lastSampleTimeStamp);
                    }
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}

