#ifndef MIRADAR_H_
#define MIRADAR_H_
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "math.h"

constexpr uint8_t CR = 0x0d;
constexpr uint8_t LF = 0x0a;
constexpr int COMM_RX_BYTE_UNIT = 64;

struct PPIData {
    int distance;
    int angle;
    int speed;
    int db;
};

struct MiRadarParam {
    explicit MiRadarParam()
        : minDistance(300),
          maxDistance(3000),
          alarmDistance(0),
          nDistance(64),
          nAngle(45),
          maxAngle(44),
          txPower(-7),
          minDb(-40),
          maxDb(-20),
          hpfGain(1),
          pgaGain(1),
          duration(100) {}

    int minDistance;
    int maxDistance;
    int alarmDistance;
    int nDistance;
    int nAngle;
    int maxAngle;
    int minAngle;
    int txPower;
    int minDb;
    int maxDb;
    int hpfGain;
    int pgaGain;
    int duration;
};

class Serial {
public:
    int fd;

    ~Serial() { close(fd); }

    int CommInit(std::string deviceFile) {
        fd = open(deviceFile.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        setupSerial();
        if (fd < 0) {
            return (-1);
        }
        return (fd);
    }

    void setupSerial() {
        struct termios tio;
        bzero((void*)&tio, (size_t)(sizeof(tio)));
        tio.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cc[VTIME] = 0;
        tio.c_cc[VMIN] = 1;

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &tio);
        fcntl(fd, F_SETFL, FNDELAY);
    }

    int CommTx(char* bpBuf, int nLen) {
        int nErr;
        nErr = write(fd, bpBuf, nLen);
        printf("TX %dbyte : %s", nLen, bpBuf);
        return (nErr);
    }
    int CommRx(char* bpBuf, int nBufSize) {
        char* bp1;
        int nErr;
        int nRxLen = 0;
        int nNopCnt = 0;
        int nReqQuit = 0;
        memset(bpBuf, 0, nBufSize);
        bp1 = bpBuf;

        while (nReqQuit == 0) {
            nErr = -1;
            while (nErr < 0) {
                nErr = read(fd, bp1, COMM_RX_BYTE_UNIT);
                //------- received
                if (0 < nErr) {
                    nNopCnt = 0;
                    bp1 += nErr;
                    nRxLen += nErr;
                    if (nBufSize <= (nRxLen + COMM_RX_BYTE_UNIT)) {
                        nErr = -1;
                        nReqQuit = 1;
                        break;
                    }
                    continue;
                }
                //------- no received
                usleep(1000);
                nNopCnt++;
                if ((0 < nRxLen) && (10 < nNopCnt)) {
                    nReqQuit = 1;
                    break;
                }
                if (1000 < nNopCnt) {
                    nReqQuit = 1;
                    break;
                }
            }
        }

        return (nRxLen);
    }
};

std::vector<std::string> split(std::string str, char del) {
    int first = 0;
    int last = str.find_first_of(del);

    std::vector<std::string> result;

    while (first < str.size()) {
        std::string subStr(str, first, last - first);

        result.push_back(subStr);

        first = last + 1;
        last = str.find_first_of(del, first);

        if (last == std::string::npos) {
            last = str.size();
        }
    }

    return result;
}

char* toCharArray(std::string str) {
    char* chrArrC = reinterpret_cast<char*>(malloc(str.size() + 1));
    strcpy(chrArrC, str.c_str());
    return chrArrC;
}

class MiRadar {
public:
    std::vector<PPIData> ppiEntries;
    Serial comm;
    char sRxBuf[65516];
    std::vector<uint8_t> map;
    int nDistance;
    int nAngle;
    int sensorState = 0;
    int prevState = 0;
    MiRadarParam radarParam;
    char sCmd[9] = {'A', ',', '0', ',', '1', ',', '0', 0x0d, 0x0a};

    explicit MiRadar() : sensorState(1) {
        radarParam.maxDistance = 3000;
        radarParam.minDistance = 300;
        radarParam.nDistance = 64;
        radarParam.alarmDistance = 1000;
        radarParam.maxAngle = 44;
        radarParam.nAngle = 45;
        radarParam.txPower = -7;
        radarParam.hpfGain = 1;
        radarParam.pgaGain = 1;
        radarParam.maxDb = -20;
        radarParam.minDb = -40;
        radarParam.duration = 200;
    }

    static double pixel2DB(int pix) {
        return static_cast<double>(pix) * 0.25 - 73.75;
    }

    void setSensorState(int state) {
        if (state <= 2 && state >= 0) {
            sCmd[4] = '0' + state;
            sensorState = state;
        }
    }

    void sendSensorMode() {
        comm.CommTx(sCmd, sizeof(sCmd));
        comm.CommRx(sRxBuf, sizeof(sRxBuf));
    }

    void stopCommunication() {
        std::string stopCommand = "A,0,0";
        stopCommand.push_back(static_cast<char>(CR));
        stopCommand.push_back(static_cast<char>(LF));
        comm.CommTx(&stopCommand[0], sizeof(stopCommand.c_str()));
        comm.CommRx(sRxBuf, sizeof(sRxBuf));
    }

    int calcDuration(int numAngle, int numDistance) {
        float a = 0.031;

        // duration = 0.031 * number of Angle * number of distance
        int duration = static_cast<int>(static_cast<float>(numAngle) *
                                        static_cast<float>(numDistance) * a);
        int firstdigits = duration % 10;
        // make the time to 5 ms chunk
        duration = (firstdigits != 0) ? duration - firstdigits + 5 : duration;
        return duration;
    }

    void setParam() {
        validateParam(radarParam);
        std::string paramCommand = generateParamCommand(radarParam);
        std::cout << paramCommand << std::endl;

        char* commandCstr = toCharArray(paramCommand);
        comm.CommTx(&paramCommand[0],
                    static_cast<int>(strlen(paramCommand.c_str())));
        comm.CommRx(sRxBuf, sizeof(sRxBuf));
    }

    void validateDistance(int& minDistance, int& maxDistance, int nDist) {
        constexpr int MAX_RANGE = 50000;

        // maximum min Distance = 256 * 25
        // this only applies when map state
        if (sensorState == 2) {
            minDistance = nDist * 25;
            minDistance = (32 * 25 > minDistance) ? 32 * 25 : minDistance;
            minDistance = (minDistance > 256 * 25) ? 256 * 25 : minDistance;
        }
        maxDistance = (maxDistance > MAX_RANGE) ? MAX_RANGE : maxDistance;
        maxDistance =
            (minDistance >= maxDistance) ? minDistance + 1000 : maxDistance;
    }

    void validateAlarmDistance(int& alarmDistance, int maxDistance) {
        constexpr int MAX_RANGE = 50000;
        alarmDistance = (alarmDistance < 0) ? 0 : alarmDistance;
        alarmDistance =
            (alarmDistance > maxDistance) ? maxDistance : alarmDistance;
        alarmDistance = (alarmDistance > MAX_RANGE) ? MAX_RANGE : alarmDistance;
    }

    void validateMaxAngle(int& maxAngle) {
        constexpr int MAX_ANGLE = 45;
        constexpr int MIN_ANGLE = 10;
        maxAngle = (maxAngle < MIN_ANGLE) ? MIN_ANGLE : maxAngle;
        maxAngle = (maxAngle > MAX_ANGLE) ? MAX_ANGLE : maxAngle;
    }

    void validateDB(int& minDB, int& maxDB) {
        constexpr int MAX_DB = -10;
        constexpr int MIN_DB = -74;
        minDB = (minDB > MAX_DB) ? MAX_DB : minDB;
        minDB = (minDB < MIN_DB) ? MIN_DB : minDB;
        maxDB = (maxDB > MAX_DB) ? MAX_DB : maxDB;
        maxDB = (maxDB < MIN_DB) ? MIN_DB + 1 : maxDB;
        minDB = (minDB >= maxDB) ? maxDB - 1 : minDB;
    }

    void validateTX(int& tx) {
        tx = (tx > 0) ? 0 : tx;
        tx = (tx < -10) ? -10 : tx;
    }

    void validateHPF(int& hpf) {
        hpf = (hpf < 0) ? 0 : hpf;
        hpf = (hpf > 2) ? 2 : hpf;
    }

    void validatePGA(int& pga) {
        pga = (pga < 0) ? 0 : pga;
        pga = (pga > 3) ? 3 : pga;
    }

    void validateNDistance(int& nDist) {
        nDist = (nDist < 32) ? 32 : nDist;
        nDist = static_cast<int>(pow(2, std::ceil(log2(nDist))));
        nDist = (nDist > 256) ? 256 : nDist;
    }

    void validateNAngle(int& nAng, int maxAngle) {
        nAng = (nAng > (maxAngle * 2 + 1)) ? maxAngle * 2 + 1 : nAng;
        nAng = (nAng < 11) ? 11 : nAng;
        nAng = ((maxAngle * 2) % ((nAng - 1)) != 0) ? maxAngle * 2 + 1 : nAng;
    }

    void validateParam(MiRadarParam& param) {
        validateNDistance(param.nDistance);
        validateDistance(param.minDistance, param.maxDistance, param.nDistance);
        validateAlarmDistance(param.alarmDistance, param.maxDistance);
        validateMaxAngle(param.maxAngle);
        validateDB(param.minDb, param.maxDb);
        validateTX(param.txPower);
        validateHPF(param.hpfGain);
        validatePGA(param.pgaGain);
        validateNAngle(param.nAngle, param.maxAngle);
        param.duration = calcDuration(param.nAngle, param.nDistance);
    }

    std::string generateParamCommand(MiRadarParam& param) {
        std::string sensorStateStr =
            (sensorState == 0) ? "17" : std::to_string(sensorState + 0x10);
        std::string paramCommand = "A,0," + sensorStateStr + ",";
        paramCommand += std::to_string(param.maxDistance) + ",";
        paramCommand += std::to_string(param.minDistance) + ",";
        paramCommand += std::to_string(param.alarmDistance) + ",";
        paramCommand += std::to_string(param.nDistance) + ",";
        paramCommand += std::to_string(param.maxAngle) + ",";
        paramCommand += std::to_string(param.nAngle) + ",";
        paramCommand += std::to_string(param.txPower) + ",";
        paramCommand += std::to_string(param.hpfGain) + ",";
        paramCommand += std::to_string(param.pgaGain) + ",";
        paramCommand += std::to_string(param.minDb) + ",";
        paramCommand += std::to_string(param.duration);
        paramCommand.push_back(CR);
        paramCommand.push_back(LF);
        return paramCommand;
    }

    void setParam(MiRadarParam param) {
        if (prevState != sensorState) {
            setSensorState(sensorState);
            sendSensorMode();
        }

        validateParam(param);

        std::string paramCommand = generateParamCommand(param);

        std::cout << paramCommand << std::endl;

        char* commandCstr = toCharArray(paramCommand);
        comm.CommTx(commandCstr, static_cast<int>(strlen(commandCstr)));
        comm.CommRx(sRxBuf, sizeof(sRxBuf));
    }

    void printParam(MiRadarParam param) {
        std::cout << "min distance : " << param.minDistance;
        std::cout << " max distance : " << param.maxDistance;
        std::cout << " duration : " << param.duration;
        std::cout << " min db : " << param.minDb;
        std::cout << " max db : " << param.maxDb;
        std::cout << " max angle : " << param.maxAngle;
        std::cout << " angle div : " << param.nAngle;
        std::cout << " distance div : " << param.nDistance << std::endl;
    }

    void setSerial(Serial& ser) { comm = ser; }

    char* getReceivedBuffer() { return sRxBuf; }

    void generatePPI(std::string& receivedBytes) {
        ppiEntries.clear();

        if (receivedBytes.find("M,1") != -1 ||
            receivedBytes.find("M,0") != -1) {
            std::vector<std::string> metadata = split(receivedBytes, ',');
            int entrynumbers = (metadata.size()) / 4;
            if (entrynumbers > 2) {
                metadata.erase(metadata.begin());
                metadata.erase(metadata.begin());
            }
            entrynumbers = (metadata.size()) / 4;

            for (int j = 0; j < entrynumbers; j++) {
                bool isNotEmpty = (std::stoi(metadata[4 * j]) |
                                   std::stoi(metadata[4 * j + 1]) |
                                   std::stoi(metadata[4 * j + 2]) |
                                   std::stoi(metadata[4 * j + 3])) != 0;
                if (isNotEmpty) {
                    PPIData ppidata;
                    ppidata.distance = std::stoi(metadata[4 * j]);
                    ppidata.angle = std::stoi(metadata[4 * j + 1]);
                    ppidata.speed = std::stoi(metadata[4 * j + 2]);
                    ppidata.db = std::stoi(metadata[4 * j + 3]);
                    ppiEntries.push_back(ppidata);
                }
            }
        }
    }

    void generateMap(std::string& receivedBytes) {
        if (receivedBytes.find("BEGIN_MAP_") != -1) {
            map.clear();
            int endIndex = receivedBytes.find("END_MAP");
            if (endIndex != -1) {
                std::string mapStr = receivedBytes.substr(18, endIndex - 18);
                std::string header = receivedBytes.substr(0, 18);
                std::string nDistanceStr;
                std::string nAngleStr;
                for (int i = 0; i < 3; i++) {
                    if (header[10 + i] != '0' || i == 2) {
                        nDistanceStr.push_back(header[10 + i]);
                    }
                    if (header[14 + i] != '0' || i == 2) {
                        nAngleStr.push_back(header[14 + i]);
                    }
                }
                nDistance = std::stoi(nDistanceStr);
                nAngle = std::stoi(nAngleStr);

                if (nDistance * nAngle == endIndex - header.size()) {
                    std::copy(mapStr.begin(), mapStr.end(),
                              std::back_inserter(map));
                } else {
                    std::cout << "map is corrupt" << std::endl;
                    std::cout << "map size is " << header.size()
                              << " , requested size is " << nDistance << " "
                              << nAngle << std::endl;
                }
            }
        }
    }

    void run() {
        if (sensorState == 0) {
            // Halt mode
            return;
        }

        int size = comm.CommRx(sRxBuf, sizeof(sRxBuf));
        std::string receivedBytes(sRxBuf, size);
        std::cout << receivedBytes << std::endl;

        if (sensorState == 1) {
            // PPI Mode
            generatePPI(receivedBytes);
        } else if (sensorState == 2) {
            // Map Mode
            generateMap(receivedBytes);
        }
        prevState = sensorState;
    }
};

#endif
