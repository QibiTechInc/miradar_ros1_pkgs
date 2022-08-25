#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>

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

constexpr int COMM_RX_BYTE_UNIT = 64;
constexpr uint8_t CR = 0x0d;
constexpr uint8_t LF = 0x0a;

struct PPIData {
    int distance;
    int angle;
    int speed;
    int db;
};

struct MiRadarParam {
    int minDb;
    int maxDb;
    int hpfGain;
    int pgaGain;
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
        bzero((void *)&tio, (size_t)(sizeof(tio)));
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

    int CommTx(char *bpBuf, int nLen) {
        int nErr;
        nErr = write(fd, bpBuf, nLen);
        printf("TX %dbyte : %s", nLen, bpBuf);
        return (nErr);
    }
    int CommRx(char *bpBuf, int nBufSize) {
        char *bp1;
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

class MiRadar {
public:
    std::vector<PPIData> ppiEntries;
    Serial comm;
    char sRxBuf[65516];
    std::vector<uint8_t> map;
    int sensorState = 0;
    char sCmd[32];
    //char sCmd[9] = {'A', ',', '0', ',', '2', ',', '0', 0x0d, 0x0a};       // original

    void setSensorState(int state) {
        switch(state) {                                                     // ST modified 2022_0705
            case 1:
                sprintf(sCmd, "START_PPI%c%c", 0x0d, 0x0a);
                sensorState = state;
                break;
            case 2:
                sprintf(sCmd, "START_MAP%c%c", 0x0d, 0x0a);
                sensorState = state;
                break;
            case 0:
                sprintf(sCmd, "B290_STOP%c%c", 0x0d, 0x0a);
                sensorState = state;
                break;
            default:
                break;
        }

        //if (state <= 2 && state >= 0) {                                   // original
        //    sCmd[4] = '0' + state;
        //    sensorState = state;
        //}
    }

    void sendSensorMode() {
        int nTxLen;

        nTxLen = strlen(sCmd);
        comm.CommTx(sCmd, nTxLen);
        comm.CommRx(sRxBuf, sizeof(sRxBuf));
    }

    void setDistance(int minDistance, int maxDistance) {
        if (minDistance > 0 && maxDistance > 0) {
            std::string distanceCommand = "B,0," + std::to_string(maxDistance) +
                                          "," + std::to_string(minDistance);
            distanceCommand.push_back((char)CR);
            distanceCommand.push_back((char)LF);
            comm.CommTx(&distanceCommand[0], sizeof(distanceCommand.c_str()));
            comm.CommRx(sRxBuf, sizeof(sRxBuf));
        }
    }

    void setAlarm(int distance) {
        if (distance > 0) {
            std::string alarmCommand = "C,0," + std::to_string(distance);
            alarmCommand.push_back((char)CR);
            alarmCommand.push_back((char)LF);
            comm.CommTx(&alarmCommand[0], sizeof(alarmCommand.c_str()));
            comm.CommRx(sRxBuf, sizeof(sRxBuf));
        }
    }

    void setParam(MiRadarParam param) {
        if (param.minDb < param.maxDb && param.hpfGain > 0 &&
            param.pgaGain > 0) {
            std::string paramCommand = "D,0," + std::to_string(param.maxDb) +
                                       "," + std::to_string(param.minDb) + "," +
                                       std::to_string(param.hpfGain) + "," +
                                       std::to_string(param.pgaGain);
            comm.CommTx(&paramCommand[0], sizeof(paramCommand.c_str()));
            comm.CommRx(sRxBuf, sizeof(sRxBuf));
        }
    }

    void setSerial(Serial &ser) { comm = ser; }

    char *getReceivedBuffer() { return sRxBuf; }

    void run() {
        if (sensorState == 0) {
            return;
        }
        int size = comm.CommRx(sRxBuf, sizeof(sRxBuf));
        std::string receivedBytes(sRxBuf, size);

        if (sensorState == 1) {
            ppiEntries.clear();

            if (receivedBytes.find("BEGIN_PPI,1") != -1 ||
                receivedBytes.find("BEGIN_PPI,0") != -1) {
                std::vector<std::string> metadata = split(receivedBytes, ',');
                int entrynumbers = ((metadata.size()) - 1) /  4;
                if (entrynumbers > 2) {
                    metadata.erase(metadata.begin());
                    metadata.erase(metadata.begin());
                }
                entrynumbers = ((metadata.size()) - 1) / 4;

                for (int j = 0; j < entrynumbers; j++) {
                    if (std::stoi(metadata[4 * j + 0]) != 0 ||
                        std::stoi(metadata[4 * j + 1]) != 0 ||
                        std::stoi(metadata[4 * j + 2]) != 0 ||
                        std::stoi(metadata[4 * j + 3]) != 0) {
                        PPIData ppidata;
                        ppidata.distance = std::stoi(metadata[4 * j + 0]);
                        ppidata.angle    = std::stoi(metadata[4 * j + 1]);
                        ppidata.speed    = std::stoi(metadata[4 * j + 2]);
                        ppidata.db       = std::stoi(metadata[4 * j + 3]);
                        ppiEntries.push_back(ppidata);

                        std::cout << "distance : " << ppidata.distance
                                  << std::endl;
                        std::cout << "angle : " << ppidata.angle << std::endl;
                        std::cout << "speed : " << ppidata.speed << std::endl;
                        std::cout << "db : " << ppidata.db << std::endl;
                    }
                }
            }
        } else if (sensorState == 2) {
            if (receivedBytes.find("BEGIN_MAP,") != -1) {
                map.clear();
                int endIndex = receivedBytes.find(",END_MAP");
                if (endIndex != -1) {
                    std::string mapStr =
                        receivedBytes.substr(18, endIndex - 18);
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
                    int nDistance = std::stoi(nDistanceStr);
                    int nAngle = std::stoi(nAngleStr);

                    std::cout << nDistance << std::endl;
                    std::cout << nAngle << std::endl;

                    if (nDistance * nAngle == endIndex - header.size()) {
                        std::cout << "Map data is not corrupted" << std::endl;
                        std::cout << mapStr << std::endl;
                        std::copy(mapStr.begin(), mapStr.end(),
                                  std::back_inserter(map));
                    }
                }
            }
        }
    }
};

int main(int argc, char *argv[]) {
    std::vector<std::string> args;

    if (argc > 1) {
        args.assign(argv + 1, argv + argc);
    } else {
        printf("Usage : ./miradar /dev/ttyUSB0 1");
    }

    std::string deviceFile = args[0];
    int sensorMode = std::stoi(args[1]);

    int fd;
    Serial comm;
    //-------- open comm port
    fd = comm.CommInit(deviceFile);
    if (fd < 0) {
        printf("can not open comm port\n");
        return (-1);
    }

    MiRadar miradar;
    miradar.setSerial(comm);

    miradar.setSensorState(sensorMode);
    miradar.sendSensorMode();

    while (true) {
        miradar.run();
    }

    return (0);
}
