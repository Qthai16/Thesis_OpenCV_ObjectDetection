#ifndef UTILITY_H
#define UTILITY_H

#include <stdio.h>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <stdint.h>
#include <fstream>
#include <time.h>

using namespace std;

class Utility
{
public:
    Utility();
    ~Utility();
    
    FILE *file1;
	const char *filename1 = "fps.txt";

    static string intToString(int number);
    static void initTime();
    static uint32_t millis();
    static uint64_t millis64();
    // uint16_t Write_To_File(uint8_t* file_data);
    void appendLineToFile(uint8_t* file_data);
    // void appendFirstLineToFile(string filepath, string file_data);
private:
    static struct timespec start_time;    
};

#endif // UTILITY_H
