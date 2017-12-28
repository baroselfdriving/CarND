#include "helpers.h"
#include "behaviour_predictor.h"
#include "behaviour_planner.h"
#include "trajectory_planner.h"
#include "waypoint.h"
#include "vehicle.h"

#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

using json = nlohmann::json;

//---------------------------------------------------------------------------------------------------------------------
int main()
//---------------------------------------------------------------------------------------------------------------------
{
  // Load waypoints from in the map from file. The waypoints lie on the innermost lane around the track.
  std::string waypointsFile = "../data/highway_map.csv";
  std::ifstream fs(waypointsFile.c_str(), std::ifstream::in);
  if(!fs.is_open())
  {
    std::cerr << "Unable to open waypoints file" << std::endl;
    return -1;
  }
  sdcnd_t3p1::WaypointList trackWaypoints;
  std::string line;
  while (std::getline(fs, line))
  {
    std::istringstream iss(line);
    sdcnd_t3p1::Waypoint wp;
    iss >> wp.point.x;
    iss >> wp.point.y;
    iss >> wp.frenet.s;
    iss >> wp.frenet.dx;
    iss >> wp.frenet.dy;
    wp.point.heading = atan2(wp.frenet.dy, wp.frenet.dx)+M_PI/2.;
    trackWaypoints.push_back(wp);
  }
  fs.close();

  if(trackWaypoints.size() < 2)
  {
    std::cerr << "No waypoints read" << std::endl;
    return -1;
  }

  // beef up the number of waypoints and make it smooth
  sdcnd_t3p1::WaypointList fineWaypoints = sdcnd_t3p1::generateFinerWaypoints(trackWaypoints, 6);
  sdcnd_t3p1::TrajectoryPlanner trajPlanner(fineWaypoints);
  sdcnd_t3p1::BehaviourPredictor predictor;
  sdcnd_t3p1::BehaviourPlanner behplanner;

  uWS::Hub h;
  bool doReset = false;
  h.onMessage([&trajPlanner, &predictor, &behplanner, &doReset](uWS::WebSocket<uWS::SERVER> ws,
              char *pData, size_t length, uWS::OpCode opCode)
  {
    (void)opCode;

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && pData[0] == '4' && pData[1] == '2')
    {
      std::string s = sdcnd_t3p1::hasJsonData(pData);

      if (s != "")
      {
        json j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          sdcnd_t3p1::Vehicle car;
          car.id = -1;
          car.position.x = j[1]["x"];
          car.position.y = j[1]["y"];
          car.position.heading = sdcnd_t3p1::deg2rad(j[1]["yaw"]);
          car.velocity.x = 0; // don't know
          car.velocity.y = 0; // don't know
          car.frenet.s = j[1]["s"];
          car.frenet.d = j[1]["d"];
          car.speed = sdcnd_t3p1::milesPerHr2metersPerSec( j[1]["speed"] );

          if(doReset)
          {
            trajPlanner.reset(car.position);
            behplanner.reset(car);
            doReset = false;
          }
          /// \note: The simulated car doesn't cover all the points passed to the sim in the last iteration. The
          /// path returned here is the waypoints left over from the last time

          // Previous path data given to the Planner
          auto previousPathX = j[1]["previous_path_x"];
          auto previousPathY = j[1]["previous_path_y"];
          sdcnd_t3p1::FrenetPoint prevPathEnd;
          prevPathEnd.s = j[1]["end_path_s"];
          prevPathEnd.d = j[1]["end_path_d"];

          size_t previousPathSz = previousPathX.size();
          if( previousPathSz != previousPathY.size())
          {
            std::cerr << "Previous path X and Y sizes not the same" << std::endl;
            return -1;
          }

          sdcnd_t3p1::CartesianPoseList previousPath;
          for(unsigned int i = 0; i < previousPathSz; ++i)
          {
            sdcnd_t3p1::CartesianPose p;
            p.x = previousPathX[i];
            p.y = previousPathY[i];
            previousPath.push_back(p);
          }

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          sdcnd_t3p1::VehicleList otherVehicles;
          for(const auto& item : sensor_fusion)
          {
            sdcnd_t3p1::Vehicle vehicle;
            vehicle.id = item[0];
            vehicle.position.x = item[1];
            vehicle.position.y = item[2];
            vehicle.position.heading = 0; // don't know, don't care
            vehicle.velocity.x = item[3];
            vehicle.velocity.y = item[4];
            vehicle.frenet.s = item[5];
            vehicle.frenet.d = item[6];
            vehicle.speed = sdcnd_t3p1::distance(0,0,vehicle.velocity.x, vehicle.velocity.y);
            otherVehicles.push_back(vehicle);
          }

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          /// ------------ PROJECT IMPLEMENTATION --------------------
          const auto predictions = predictor.predict(car, otherVehicles);
          auto behaviour = behplanner.compute(predictions, car);
          const auto path = trajPlanner.computePlan(behaviour.targetLane, car, otherVehicles, previousPath);
          for(const auto& item : path)
          {
            next_x_vals.push_back( item.x );
            next_y_vals.push_back( item.y );
          }
          /// ------------ END PROJECT IMPLEMENTATION --------------------

/*
          /// ------------ TEST --------------------
          double dist_inc = 0.5;
          for(int i = 0; i < 50; i++)
          {
            sdcnd_t3p1::FrenetPoint nextF;
            nextF.s = car.frenet.s + (i+1) * dist_inc;
            nextF.d = 6;

            sdcnd_t3p1::CartesianPose xy = sdcnd_t3p1::getCartesianFromFrenet(nextF.s, nextF.d, fineWaypoints);

            next_x_vals.push_back(xy.x);
            next_y_vals.push_back(xy.y);
          }
          /// ------------- END TEST -------------------
*/

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //std::this_thread::sleep_for(std::chrono::milliseconds(500));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    (void)data;
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h, &doReset](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    (void)ws;
    (void)req;
    doReset = true;
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
  {
    (void)code;
    (void)message;
    (void)length;
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
