#include "miradar.h"
#include <string>
#include <gtest/gtest.h>

TEST(COMMINIT, DEVTTYUSB0){
    Serial serial;

    #ifdef __linux__ 
    std::string devicefile = "/dev/ttyUSB0";
    #endif

    int fd = serial.CommInit(devicefile);
    

    FILE *fp = fopen("ls /dev | grep ttyUSB", "r");
    EXPECT_FALSE(fp == NULL);
    
    std::string result;
    std::array<char, 128> buffer;
    while (fgets(buffer.data(), 128, fp) != NULL) {
        result += buffer.data();
    }

    if(fd >= 0){
        //if found, then /dev/ttyUSB0 should be in the environment
        ASSERT_TRUE(result.find(devicefile) != -1);
    }
    else{
        //if not found, then /dev/ttyUSB0 is not in the environment
        ASSERT_TRUE(result.find(devicefile) == -1);
    }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}