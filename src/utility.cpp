#include "../inc/utility.h"

struct timespec Utility::start_time;

Utility::Utility()
{

}

Utility::~Utility()
{

}

string Utility::intToString(int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void Utility::initTime()
{
    clock_gettime(CLOCK_MONOTONIC, &start_time);
}

uint32_t Utility::millis()
{
    return millis64() & 0xFFFFFFFF;
}

uint64_t Utility::millis64()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) -
        (start_time.tv_sec +
            (start_time.tv_nsec*1.0e-9)));
}

// uint16_t Utility::Write_To_File(uint8_t* file_data)
// {
//     //OPEN TXT FILE IN OUR APPLICAITONS DIRECTORY OR CREATE IT IF IT DOESN'T EXIST
// 	file1 = fopen(filename1, "a"); //open the file for appending (writing at the end of file)
// 	if (file1) //----- FILE EXISTS -----
// 	{
//         // long int file_index = ftell(file1);
//         // fwrite(file_data, sizeof(unsigned char), sizeof(file_data), file1);
// 		fread(file_data, sizeof(unsigned char), sizeof(file_data), file1);
		
// 		fclose(file1);
// 		file1 = NULL;
// 	}
// 	else //----- FILE NOT FOUND -----
// 	{
// 		printf("File not found\n");
// 		file1 = fopen(filename1, "w"); //Write new file
// 		if (file1)
// 		{
// 			printf("Writing new file\n");
// 			fwrite(file_data, sizeof(unsigned char), sizeof(file_data), file1);

// 			fclose(file1);
// 			file1 = NULL;
// 		}
// 	}
    
// }

void Utility::appendLineToFile(uint8_t* file_data)
{
    std::ofstream file;
    //can't enable exception now because of gcc bug that raises ios_base::failure with useless message
    // file.exceptions(file.exceptions() | std::ios::failbit);
    file.open("fps.txt", std::ios_base::app);
    // if (file.fail())
    //     throw std::ios_base::failure(std::strerror(errno));

    //make sure write fails with exception if something is wrong
    // file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);
    file << file_data;
}

// void Utility::appendFirstLineToFile(string filepath_1, string file_data__1)
// {
//     std::ofstream outfile;
//     outfile.open(filepath_1, std::ios_base::app);
//     // if (file.fail())
//     //     throw std::ios_base::failure(std::strerror(errno));

//     // file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);
//     outfile << file_data__1;
// }
