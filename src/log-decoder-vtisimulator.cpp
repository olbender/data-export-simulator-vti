/*
 * Copyright (C) 2018 Ola Benderius, Björnborg Nguyen
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

// #include <Eigen/Dense>
#include "decoder.hpp"
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") || 
      0 == commandlineArguments.count("freq") ||
      0 == commandlineArguments.count("data")){
    std::cerr << argv[0] << " Neuromuscular model." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --freq=<Frequency> --cid=<od4 session>  --data=<data bin dat>[--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --freq=100 --cid=111 --data=data.bin" << std::endl;
    retCode = 2;
    return retCode;
  } 

  // bool const VERBOSE{commandlineArguments.count("verbose") != 0};
  uint16_t const CID = std::stoi(commandlineArguments["cid"]);
  float const FREQ = std::stof(commandlineArguments["freq"]);
  std::string const DATA = commandlineArguments["data"];
  // double const DT = 1 / FREQ;
  cluon::OD4Session od4{CID};

  // od4.send(test, cluon::time::now(), 0);
  // auto onPressureRequest{[&biceps](cluon::data::Envelope &&envelope)
  // {
  //   if (envelope.senderStamp() == 1) {
  //     biceps.Stimulate();
  //   }
  // }};
  // od4.dataTrigger(opendlv::proxy::PressureRequest::ID(), onPressureRequest);

  Decoder data(DATA);
  std::vector<float> dataVec = data.decodeDat();

  cluon::data::TimeStamp t0 = cluon::time::now();
  auto atFrequency{[&od4, &t0, &data, &dataVec]() -> bool
    {
      cluon::data::TimeStamp now = cluon::time::now();
      double time = ((double) now.seconds() + (double) now.microseconds()/1000000.0 - (double) t0.seconds() + (double) t0.microseconds()/1000000.0);

      
      // opendlv::proxy::PressureReading tensionMsg;
      // tensionMsg.pressure(biceps.GetForcef());
      // od4.send(tensionMsg, cluon::time::now(), 0);
      // std::cout << "time1: " << time << std::endl;
      // std::cout << "time2: " << dataVec.at(0) << std::endl;
      while (time > dataVec.at(0)) {
        dataVec = data.decodeDat();
        // std::cout << "Decoded data" << std::endl;
        // for(auto it = dataVec.cbegin(); it != dataVec.cend(); it++) {
        //   std::cout << *it << " ";
        // }
        // std::cout << std::endl;
        {
          opendlv::sim::Frame m;
          m.x(dataVec.at(6));
          m.y(dataVec.at(7));
          m.z(dataVec.at(8));
          // m.roll(dataVec.at(6));
          // m.pitch(dataVec.at(6));
          m.yaw(dataVec.at(5));
          od4.send(m);
        }
        {
          opendlv::sim::KinematicState m;
          m.vx(dataVec.at(13));
          m.vy(dataVec.at(14));
          m.rollRate(dataVec.at(17));       
          m.pitchRate(dataVec.at(18));        
          m.yawRate(dataVec.at(19));
          od4.send(m);           
        }


//     simulationTime = v[0];
//     roadId = v[1];
//     roadPositionFrontAxleLongitudinal = v[2];
//     roadPositionFrontAxleLateral = v[3];
//     roadPositionHeading = v[4];
//     globalHeading = v[5];
//     globalPositionFrontAxleX = v[6];
//     globalPositionFrontAxleY = v[7];
//     globalPositionFrontAxleZ = v[8];
//     inputThrottlePosition = v[9];
//     inputBrakePressure = v[10];
//     inputBrakeForce = v[11];
//     inputSteeringWheelAngle = v[12];
//     localVelocityCogX = v[13];
//     localVelocityCogY = v[14];
//     localAccelerationCogX = v[15];
//     localAccelerationCogY = v[16];
//     localAngularVelocityRoll = v[17];
//     localAngularVelocityPitch = v[18];
//     localAngularVelocityYaw = v[19];
//     oveAxMc = v[20];
//     oveAyMc = v[21];
//     oveAzMc = v[22];
//     scenarioEventId = v[23];
//     scenarioStateId = v[24];
//     scenarioStateTimer = v[25];
//     torqueMotorRef = v[26];
//     torqueAligning = v[27];
//     torqueMotorFeedback = v[28];
//     torqueDamping = v[29];
//     torqueFriction = v[30];
//     torqueDriver = v[31];
//     scenarioTorque = v[32];
//     scenarioSlideY = v[33];
//     scenarioSlideYD = v[34];
//     scenarioSlideYDD = v[35];
//     scenarioDistractionTriggered = v[36];
//     motioncueYSd = v[37];
//     motioncueYSdD = v[38];
//     motioncueYSdDD = v[39];
//     motioncueYHx = v[40];
//     motioncueYHxD = v[41];
//     motioncueYHxDD = v[42];
//     emgOverview1 = v[43];
//     emgOverview2 = v[44];
//     emgOverview3 = v[45];
//     emgOverview4 = v[46];
//     emgOverview5 = v[47];
//     emgOverview6 = v[48];
//     emgOverview7 = v[49];
//     emgOverview8 = v[50];
        
      }

      return true;
    }};

  od4.timeTrigger(FREQ, atFrequency);


  return retCode;
}
