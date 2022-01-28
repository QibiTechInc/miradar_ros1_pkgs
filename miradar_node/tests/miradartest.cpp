#include <gtest/gtest.h>
#include <miradar.h>
#include <miradar_helpers.h>


/**
 * Test case for parameter set command generation
 **/
TEST(GENERATEPARAMCOMMAND, GENERAL){
  constexpr int REGRESSION_TEST_TRIALS  = 100;
  MiRadar miradar;
  std::random_device rnd;
  std::mt19937 mt(rnd());     
  std::uniform_int_distribution<> randSensorState(1, 2); 

  for(int i = 0; i < REGRESSION_TEST_TRIALS; i++){
    MiRadarParam miradarParam;
    
    miradarParam = generateRandomRadarParamInBounds();
    miradar.validateParam(miradarParam);
    miradar.setSensorState(randSensorState(mt));

    int sensorState = randSensorState(mt);
    miradar.setSensorState(sensorState);

    std::string paramCommand = miradar.generateParamCommand(miradarParam);

    //remove CRLF
    paramCommand.pop_back();
    paramCommand.pop_back();


    std::vector<std::string> paramList = split(paramCommand, ',');


    constexpr int HEADER1 = 0;
    constexpr int HEADER2 = 1;
    constexpr int SENSOR = 2;
    constexpr int MAXD = 3;
    constexpr int MIND = 4;
    constexpr int ALARM =5;
    constexpr int NDIST = 6;
    constexpr int MAXANGLE = 7;
    constexpr int NANGLE = 8;
    constexpr int TX = 9;
    constexpr int HPF = 10;
    constexpr int PGA = 11;
    constexpr int MINDB = 12;
    constexpr int DUR = 13;


    EXPECT_TRUE(paramList[HEADER1] == "A");
    EXPECT_TRUE(paramList[HEADER2] == "0");
    EXPECT_TRUE(std::stoi(paramList[SENSOR]) == miradar.sensorState + 0x10) << "actual : " << paramList[SENSOR] << "expected: " <<miradar.sensorState + 0x10;
    EXPECT_TRUE(std::stoi(paramList[MAXD]) == miradarParam.maxDistance);
    EXPECT_TRUE(std::stoi(paramList[MIND]) == miradarParam.minDistance);
    EXPECT_TRUE(std::stoi(paramList[ALARM]) == miradarParam.alarmDistance);
    EXPECT_TRUE(std::stoi(paramList[NDIST]) == miradarParam.nDistance);
    EXPECT_TRUE(std::stoi(paramList[MAXANGLE]) == miradarParam.maxAngle);
    EXPECT_TRUE(std::stoi(paramList[NANGLE]) == miradarParam.nAngle);
    EXPECT_TRUE(std::stoi(paramList[TX]) == miradarParam.txPower);
    EXPECT_TRUE(std::stoi(paramList[HPF]) == miradarParam.hpfGain);
    EXPECT_TRUE(std::stoi(paramList[PGA]) == miradarParam.pgaGain);
    EXPECT_TRUE(std::stoi(paramList[MINDB]) == miradarParam.minDb);
    EXPECT_TRUE(std::stoi(paramList[DUR]) == miradarParam.duration) << "actual : " << paramList[DUR] << " expected: " << miradarParam.duration << std::endl;

  }
    /*
    EXPECT_TRUE(miradarParamCp.maxDistance == miradarParam.maxDistance) << "actual : " << miradarParamCp.maxDistance << " expected: " << miradarParam.maxDistance << std::endl;
    EXPECT_TRUE(miradarParamCp.minDistance == miradarParam.minDistance)<< "actual : " << miradarParamCp.minDistance << " expected: " << miradarParam.minDistance << std::endl;
    EXPECT_TRUE(miradarParamCp.alarmDistance == miradarParam.alarmDistance) << "actual : " << miradarParamCp.alarmDistance << " expected: " << miradarParam.alarmDistance << std::endl;
    EXPECT_TRUE(miradarParamCp.maxAngle == miradarParam.maxAngle) << "actual : " << miradarParamCp.maxAngle << " expected: " << miradarParam.maxAngle << std::endl;
    EXPECT_TRUE(miradarParamCp.nDistance == miradarParam.nDistance) << "actual : " << miradarParamCp.nDistance << " expected: " << miradarParam.nDistance << std::endl;
    EXPECT_TRUE(miradarParamCp.nAngle == miradarParam.nAngle) << "actual : " << miradarParamCp.nAngle << " expected: " << miradarParam.nAngle << " max Angle : " <<  miradarParam.maxAngle << std::endl;
    EXPECT_TRUE(miradarParamCp.txPower == miradarParam.txPower) << "actual : " << miradarParamCp.txPower << " expected: " << miradarParam.txPower << std::endl;
    EXPECT_TRUE(miradarParamCp.hpfGain == miradarParam.hpfGain) << "actual : " << miradarParamCp.hpfGain << " expected: " << miradarParam.hpfGain << std::endl;
    EXPECT_TRUE(miradarParamCp.pgaGain == miradarParam.pgaGain) << "actual : " << miradarParamCp.pgaGain << " expected: " << miradarParam.pgaGain << std::endl;
    EXPECT_TRUE(miradarParamCp.duration == miradarParam.duration) << "actual : " << miradarParamCp.duration << " expected: " << miradarParam.duration << std::endl;
    */
}

/**
 * Test case for all sensorstate including boundary
 **/
TEST(SETSENSORSTATE, ALL){
  MiRadar miradar; 
  constexpr int lowerBounds = -1, haltState = 0, ppiState = 1, mapState = 2, upperBounds = 3;

  int lastState = miradar.sensorState;
  miradar.setSensorState(lowerBounds);
  EXPECT_TRUE(miradar.sensorState == lastState && miradar.sensorState != lowerBounds);
  miradar.setSensorState(haltState);
  EXPECT_TRUE(miradar.sensorState == haltState);
  miradar.setSensorState(ppiState);
  EXPECT_TRUE(miradar.sensorState == ppiState);
  miradar.setSensorState(mapState);
  EXPECT_TRUE(miradar.sensorState == mapState);
  lastState = miradar.sensorState;
  miradar.setSensorState(upperBounds);
  EXPECT_TRUE(miradar.sensorState == lastState && miradar.sensorState != upperBounds);
}


/**
 * Test case for bound maxDistance
 **/

TEST(VALIDATEPARAM, MAXDISTANCE){
  MiRadar miradar;
  MiRadarParam case1, case2, case3;

  constexpr int UPPER_BOUNDS = 50001;
  constexpr int LOWER_BOUNDS = -1;
  constexpr int MIN_BIGGER = 256 * 25;

  case1.maxDistance = UPPER_BOUNDS;
  case2.maxDistance = LOWER_BOUNDS;
  case3.maxDistance = MIN_BIGGER - 1;
  case3.minDistance = MIN_BIGGER;
  miradar.validateDistance(case1.minDistance, case1.maxDistance, 64);
  EXPECT_TRUE(case1.maxDistance == 50000);
  miradar.validateDistance(case2.minDistance, case2.maxDistance, 64);
  EXPECT_TRUE(case2.maxDistance > case2.minDistance);
  EXPECT_TRUE(case2.maxDistance > 0);
  miradar.validateDistance(case3.minDistance, case3.maxDistance, 256);
  EXPECT_TRUE(case3.maxDistance > case2.minDistance);
}


TEST(VALIDATEPARAM, MAXANGLE){
  MiRadar miradar;
  MiRadarParam case1, case2, case3, case4;

  constexpr int UPPER_BOUNDS = 45;
  constexpr int LOWER_BOUNDS = 10;

  case1.maxAngle = UPPER_BOUNDS + 1;
  case2.maxAngle = UPPER_BOUNDS - 1;
  case3.maxAngle = LOWER_BOUNDS + 1;
  case4.maxAngle = LOWER_BOUNDS - 1;
  miradar.validateMaxAngle(case1.maxAngle);
  miradar.validateMaxAngle(case2.maxAngle);
  miradar.validateMaxAngle(case3.maxAngle);
  miradar.validateMaxAngle(case4.maxAngle);
  EXPECT_TRUE(case1.maxAngle == UPPER_BOUNDS);
  EXPECT_TRUE(case2.maxAngle == UPPER_BOUNDS - 1);
  EXPECT_TRUE(case3.maxAngle == LOWER_BOUNDS + 1);
  EXPECT_TRUE(case4.maxAngle == LOWER_BOUNDS) << "actual :" << case4.maxAngle << " expected : " << LOWER_BOUNDS;

}

/**
 * boundary test for minimum distance
 **/
TEST(VALIDATEPARAM, MINDISTANCE){
  MiRadar miradar;
  MiRadarParam case1, case2, case3, case4;

  constexpr int UPPER_BOUNDS = 50001;
  constexpr int LOWER_BOUNDS = -1;
  constexpr int MIN_BIGGER = 256 * 25;



  case1.minDistance = 32 * 25  -1 ;
  miradar.validateDistance(case1.minDistance ,case2.maxDistance, 32);
  EXPECT_TRUE(case1.minDistance == 32 * 25);

  case2.minDistance = 256 * 25 + 1;
  miradar.validateDistance(case2.minDistance, case2.maxDistance, 256);
  EXPECT_TRUE(case2.minDistance == 256 * 25);

  case3.minDistance = 3333;
  miradar.validateDistance(case3.minDistance, case3.maxDistance, 256);
  EXPECT_TRUE(case3.minDistance == 256 * 25);

  case4.minDistance = 256 * 25;
  case4.maxDistance = 1000;
  miradar.validateDistance(case4.minDistance, case4.maxDistance, 256);
  EXPECT_TRUE(case4.maxDistance > case4.minDistance);
}

/**
 * boundary test for nDistance
 **/

TEST(VALIDATEPARAM, NDISTANCE){
  MiRadar miradar;
  MiRadarParam case1, case2, case3;

  constexpr int MAX_NDIST = 256;
  constexpr int MIN_NDIST = 32;

  case1.nDistance = MAX_NDIST * 2;
  miradar.validateNDistance(case1.nDistance);
  EXPECT_TRUE(case1.nDistance == MAX_NDIST);

  case2.nDistance = MIN_NDIST/2;
  miradar.validateNDistance(case2.nDistance);
  EXPECT_TRUE(case2.nDistance == MIN_NDIST);

  case3.nDistance = 40;
  miradar.validateNDistance(case3.nDistance);
  EXPECT_TRUE(case3.nDistance == 64);
}

/**
 * boundary test for DB
 **/

TEST(VALIDATEPARAM, DB){
  MiRadar miradar;
  MiRadarParam case1, case2, case3, case4;

  constexpr int MIN_DB = -74;
  constexpr int MAX_DB = -10;

  case1.minDb = -100;
  case1.maxDb = -20;
  miradar.validateDB(case1.minDb, case1.maxDb);
  EXPECT_TRUE(case1.minDb == MIN_DB) << "actual: " << case1.minDb << "maxdb : " << case1.maxDb << std::endl;

  case2.maxDb = -9;
  miradar.validateDB(case2.minDb, case2.maxDb);
  EXPECT_TRUE(case2.maxDb == MAX_DB) << "actual: " << case2.maxDb << std::endl;


  case3.minDb = -20;
  case3.maxDb = -40;
  miradar.validateDB(case3.minDb, case3.maxDb);
  EXPECT_TRUE(case3.maxDb > case3.minDb);
}


/**
 * boundary test for nangle
 **/
TEST(VALIDATEPARAM, NANGLE){
  MiRadar miradar;
  MiRadarParam case1, case2, case3, case4;

  constexpr int MAX_ANGLE = 45;
  constexpr int MAX_NANGLE = MAX_ANGLE * 2 + 1;
  constexpr int MIN_NANGLE = 11;

  case1.nAngle = 20;
  miradar.validateNAngle(case1.nAngle, MAX_ANGLE);
  EXPECT_TRUE(case1.nAngle == MAX_NANGLE);


  case2.nAngle = 10;
  miradar.validateNAngle(case2.nAngle, MAX_ANGLE);
  EXPECT_TRUE(case2.nAngle == MIN_NANGLE);

  case3.nAngle = 30;
  case3.maxAngle = 60;
  miradar.validateNAngle(case3.nAngle, case3.maxAngle);
  EXPECT_TRUE(case3.nAngle == case3.maxAngle * 2 + 1);
}

/**
 * Boundary test for PGA
 **/
TEST(VALIDATEPARAM, PGA){
  MiRadar miradar;
  MiRadarParam case1, case2, case3, case4;

  constexpr int MAX_PGA = 3;
  constexpr int MIN_PGA = 0;

  case1.pgaGain = MAX_PGA + 1;
  case2.pgaGain = MAX_PGA - 1;
  case3.pgaGain = MIN_PGA - 1;
  case4.pgaGain = MIN_PGA + 1;

  miradar.validatePGA(case1.pgaGain);
  miradar.validatePGA(case2.pgaGain);
  miradar.validatePGA(case3.pgaGain);
  miradar.validatePGA(case4.pgaGain);

  EXPECT_TRUE(case1.pgaGain == MAX_PGA);
  EXPECT_TRUE(case2.pgaGain == MAX_PGA - 1);
  EXPECT_TRUE(case3.pgaGain == MIN_PGA);
  EXPECT_TRUE(case4.pgaGain == MIN_PGA + 1);
}

/**
 * Boundary test for HPF
 **/
TEST(VALIDATEPARAM, HPF){
  MiRadar miradar;
  MiRadarParam case1, case2, case3, case4;

  constexpr int MAX_HPF = 2;
  constexpr int MIN_HPF = 0;

  case1.hpfGain = MAX_HPF + 1;
  case2.hpfGain = MAX_HPF - 1;
  case3.hpfGain = MIN_HPF - 1;
  case4.hpfGain = MIN_HPF + 1;

  miradar.validateHPF(case1.hpfGain);
  miradar.validateHPF(case2.hpfGain);
  miradar.validateHPF(case3.hpfGain);
  miradar.validateHPF(case4.hpfGain);

  EXPECT_TRUE(case1.hpfGain == MAX_HPF);
  EXPECT_TRUE(case2.hpfGain == MAX_HPF - 1);
  EXPECT_TRUE(case3.hpfGain == MIN_HPF);
  EXPECT_TRUE(case4.hpfGain == MIN_HPF + 1);
}

/**
 * Boundary Test for TX
 **/
TEST(VALIDATEPARAM, TX){
  MiRadar miradar;
  MiRadarParam case1, case2, case3, case4;

  constexpr int MAX_TX = 0;
  constexpr int MIN_TX = -10;

  case1.txPower = MAX_TX + 1;
  case2.txPower = MAX_TX - 1;
  case3.txPower = MIN_TX - 1;
  case4.txPower = MIN_TX + 1;

  miradar.validateTX(case1.txPower);
  miradar.validateTX(case2.txPower);
  miradar.validateTX(case3.txPower);
  miradar.validateTX(case4.txPower);

  EXPECT_TRUE(case1.txPower == MAX_TX);
  EXPECT_TRUE(case2.txPower == MAX_TX - 1);
  EXPECT_TRUE(case3.txPower == MIN_TX);
  EXPECT_TRUE(case4.txPower == MIN_TX + 1);
}

/**
 * Test case for general param validation test.
 **/
TEST(VALIDATEPARAM, GENERAL){
  constexpr int REGRESSION_TEST_TRIALS  = 100;
  MiRadar miradar;
  for(int i = 0; i < REGRESSION_TEST_TRIALS; i++){
    MiRadarParam miradarParam, miradarParamCp;
    miradarParam = generateRandomRadarParamInBounds();
    miradarParamCp = miradarParam;
    //reference so we copy
    miradar.validateParam(miradarParamCp);
    EXPECT_TRUE(miradarParamCp.maxDistance == miradarParam.maxDistance) << "actual : " << miradarParamCp.maxDistance << " expected: " << miradarParam.maxDistance << std::endl;
    EXPECT_TRUE(miradarParamCp.minDistance == miradarParam.minDistance)<< "actual : " << miradarParamCp.minDistance << " expected: " << miradarParam.minDistance << std::endl;
    EXPECT_TRUE(miradarParamCp.alarmDistance == miradarParam.alarmDistance) << "actual : " << miradarParamCp.alarmDistance << " expected: " << miradarParam.alarmDistance << std::endl;
    EXPECT_TRUE(miradarParamCp.maxAngle == miradarParam.maxAngle) << "actual : " << miradarParamCp.maxAngle << " expected: " << miradarParam.maxAngle << std::endl;
    EXPECT_TRUE(miradarParamCp.nDistance == miradarParam.nDistance) << "actual : " << miradarParamCp.nDistance << " expected: " << miradarParam.nDistance << std::endl;
    EXPECT_TRUE(miradarParamCp.nAngle == miradarParam.nAngle) << "actual : " << miradarParamCp.nAngle << " expected: " << miradarParam.nAngle << " max Angle : " <<  miradarParam.maxAngle << std::endl;
    EXPECT_TRUE(miradarParamCp.txPower == miradarParam.txPower) << "actual : " << miradarParamCp.txPower << " expected: " << miradarParam.txPower << std::endl;
    EXPECT_TRUE(miradarParamCp.hpfGain == miradarParam.hpfGain) << "actual : " << miradarParamCp.hpfGain << " expected: " << miradarParam.hpfGain << std::endl;
    EXPECT_TRUE(miradarParamCp.pgaGain == miradarParam.pgaGain) << "actual : " << miradarParamCp.pgaGain << " expected: " << miradarParam.pgaGain << std::endl;
    EXPECT_TRUE(miradarParamCp.duration == miradarParam.duration) << "actual : " << miradarParamCp.duration << " expected: " << miradarParam.duration << std::endl;
  }
}


/**
 * Test case for boundary value (0,0) map.
 **/
TEST(GENERATEMAP, BOUNDARY){
  std::string receivedBytes = MapGenerator(0, 0);
  MiRadar miradar;
  miradar.generateMap(receivedBytes);
  EXPECT_TRUE(miradar.map.size() == 0) << "size : " << miradar.map.size() << "expected : " << 0 << std::endl << "received bytes: " << receivedBytes;
}

/**
 * Test case for general map data
 **/
TEST(GENERATEMAP, GENERAL){
  constexpr int distance_div[4] = {32, 64, 128, 256};
  //constexpr int max_angle[4] = {10, 15, 30, 45};
  constexpr int angle_div[4] = {21, 31, 61, 91};
  std::string receivedBytes;
  MiRadar miradar;

  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      receivedBytes = MapGenerator(distance_div[i], angle_div[j]);
      miradar.generateMap(receivedBytes);
      EXPECT_TRUE(miradar.map.size() == distance_div[i] * angle_div[j]) << "size :" << miradar.map.size() << " expected : " << distance_div[i] * angle_div[j] << std::endl << "received bytes: " << receivedBytes;
    }
  }
}



/***
 * Test case for maximum byte entry of PPI Data.
 **/
TEST(GENERATEPPI, BOUNDARY2){
  std::string receivedBytes;
  MiRadar miradar;
  std::string header[2] = {"M,0,", "M,1,"};

  constexpr int MIN = 0;
  constexpr int MAX = 1;

  int distance[2] = {1, 50000};
  int speed[4] = {1, 128, -1, -127};
  int angle[4] = {1, 90, -1, -90};
  int db[2] = {-1, -80};

  for(int i = 0; i < 2; i++){
    for(int j = 0; j < 4; j++){
      for(int k = 0; k < 4; k++){
        for(int l = 0; l < 2; l++){
            std::string distanceField = std::to_string(distance[i]) + ",";
            std::string angleField = std::to_string(angle[j]) + ",";
            std::string speedField = std::to_string(speed[k]) +  ",";
            std::string dbField = std::to_string(db[l]) + ",";
            std::string entryMsg = distanceField +  angleField + speedField + dbField;
            for(int m = 0; m < 2; m++){
              std::string data;
              for(int n = 0; n < 8; n++){
                data += entryMsg;
              }
              std::string headerField = header[m];
              receivedBytes = headerField + data;
              miradar.generatePPI(receivedBytes);
              EXPECT_TRUE(miradar.ppiEntries.size() == MAX_NUM_PPI) << "message : " << receivedBytes << ", size is " << miradar.ppiEntries.size() <<  " expected " << MAX_NUM_PPI << std::endl;
              for(int o=0; o < 8; o++){
                EXPECT_TRUE(miradar.ppiEntries[o].distance == distance[i]);
                EXPECT_TRUE(miradar.ppiEntries[o].angle == angle[j]);
                EXPECT_TRUE(miradar.ppiEntries[o].speed == speed[k]);
                EXPECT_TRUE(miradar.ppiEntries[o].db == db[l]);
              }

            }
        }
      }
    }
  }
}


/***
 * Test case for 0,0,0,1, .......,0,0,0,1 zero distance of entries.
 **/
TEST(GENERATEPPI, BOUNDARY1){
    std::string receivedBytes;
    MiRadar miradar;

    

  for(int i = 0; i < 4; i++){
      receivedBytes = BoundaryPPIGenerator(0, MAX_NUM_PPI, i);
      miradar.generatePPI(receivedBytes);
      EXPECT_TRUE(miradar.ppiEntries.size() == MAX_NUM_PPI) << "message : " << receivedBytes << ", size is " << miradar.ppiEntries.size() <<  " expected " << MAX_NUM_PPI << std::endl;
      for(int j = 0; j < miradar.ppiEntries.size(); j++){
        if(i == 0){
          EXPECT_FALSE(miradar.ppiEntries[j].distance == 0);
          EXPECT_FALSE(miradar.ppiEntries[j].angle == 0);
          EXPECT_FALSE(miradar.ppiEntries[j].speed == 0);
          EXPECT_FALSE(miradar.ppiEntries[j].db == 0);
        }else if(i == 1){
          EXPECT_TRUE(miradar.ppiEntries[j].distance == 0);
          EXPECT_FALSE(miradar.ppiEntries[j].angle == 0);
          EXPECT_FALSE(miradar.ppiEntries[j].speed == 0);
          EXPECT_FALSE(miradar.ppiEntries[j].db == 0);

        }else if(i == 2){
          EXPECT_TRUE(miradar.ppiEntries[j].distance == 0);
          EXPECT_TRUE(miradar.ppiEntries[j].angle == 0);
          EXPECT_FALSE(miradar.ppiEntries[j].speed == 0);
          EXPECT_FALSE(miradar.ppiEntries[j].db == 0);

        }else if(i == 3){
          EXPECT_TRUE(miradar.ppiEntries[j].distance == 0);
          EXPECT_TRUE(miradar.ppiEntries[j].angle == 0);
          EXPECT_TRUE(miradar.ppiEntries[j].speed == 0);
          EXPECT_FALSE(miradar.ppiEntries[j].db == 0);
        }
      }
  }
}


/***
 * Test case for Artificial General PPI Data.
 **/
TEST(GENERATEPPI, GENERAL){

    std::string receivedBytes;
    MiRadar miradar;

    constexpr int MAX_NUM_PPI = 8;

    for(int i = 0; i < MAX_NUM_PPI; i++){
      receivedBytes = PPIGenerator(0, i, MAX_NUM_PPI, true);
      miradar.generatePPI(receivedBytes);
      EXPECT_TRUE(miradar.ppiEntries.size() == i) << "message : " << receivedBytes << ", size is " << miradar.ppiEntries.size() <<  " expected " << i << std::endl;

      //receivedBytes = PPIGenerator(1, i, 6, true);
      //miradar.generatePPI(receivedBytes);
      //EXPECT_TRUE(miradar.ppiEntries.size() == i);
    }
}

/**
 * Test case for the PDF sample value
 **/
TEST(GENERATEPPI, PDFSAMPLE){
    std::string receivedBytes = "M,0,1700,-30,-4,-38,0,0,0,0,5916,-18,-2,-46,0,0,0,0,0,0,0,0,0,0,0,0";
    receivedBytes.push_back(CR);
    receivedBytes.push_back(LF);
    MiRadar miradar;
    miradar.generatePPI(receivedBytes);
    EXPECT_TRUE(miradar.ppiEntries.size() == 2) << "ppi entry size was " << miradar.ppiEntries.size();

    PPIData ppi1, ppi2;
    ppi1 = miradar.ppiEntries[0];
    ppi2 = miradar.ppiEntries[1];

    EXPECT_TRUE(ppi1.distance == 1700) << "ppi distance was " << ppi1.distance;
    EXPECT_TRUE(ppi1.angle == -30) << "ppi angle was " << ppi1.angle;
    EXPECT_TRUE(ppi1.speed == -4) << "ppi speed was " << ppi1.speed;
    EXPECT_TRUE(ppi1.db == -38)<< "ppi db was " << ppi1.db;

    EXPECT_TRUE(ppi2.distance == 5916) << "ppi distance was " << ppi2.distance;
    EXPECT_TRUE(ppi2.angle == -18) << "ppi angle was " << ppi2.angle;
    EXPECT_TRUE(ppi2.speed == -2) << "ppi speed was " << ppi2.speed;
    EXPECT_TRUE(ppi2.db == -46) << "ppi db was " << ppi2.db;
}




int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}