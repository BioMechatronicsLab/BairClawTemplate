/*******************************************************************************

 Name : Randy Hellman
 Date : Jan. 7, 2014
 Description:
 
    	BairClawDataPlayBack.h reads in a binary file of data to be tracked 
		and can then be used to replay that data to track a trakectory.
		The constructor opens the file and then you need to call readFile() to
		pull the data to the file into a block of memory on the heap.

*******************************************************************************/


#ifndef BAIRCLAWDATAPLAYBACK_H
#define BAIRCLAWDATAPLAYBACK_H
#include <iostream>
#include <fstream>
#include <string>

class dataRecording {
	std::streampos size;


public:
	int dataCount;
	bool ready;
	int* data;
	//Constructor initilizes values
	dataRecording() : size(0), ready(0){
		data = NULL;
		std::cout << "dataRecording constructor* SIZE - " << size << std::endl; 
	}
	~dataRecording() {
		size = 0; 
		ready = 0;
		delete []data;
		std::cout << "dataRecording destructor* SIZE - " << size << std::endl; 
	}
	friend std::ostream&
	operator<<(std::ostream& os, const dataRecording& t) {
		return os << std::endl << "File Size 	     - " << t.size << " kB" 
				<< std::endl << "numOfDataPoints(int) - " 
				<< t.size/sizeof(int) << std::endl << std::endl;

	}
	void readFile(const char* fname){
		std::ifstream file(fname, std::ios::in|std::ios::binary|std::ios::ate);
		if (file.is_open())
  		{
		    size = file.tellg(); 
		    dataCount = (size/sizeof(int));
		    data = new int[dataCount];
		    
		    file.seekg (0, std::ios::beg);
		    file.read ((char*)data, size);
		    file.close();

		    ready = true;
		    std::cout << "the entire file content is in memory" << std::endl;  
		}
		else 
		{
			std::cout << "Unable to open file";
			ready = false;
		}
	}
	void dispData(){
			std::cout << "fileSize read - " << size << " kB, DataPoints - " << dataCount << std::endl;
		    for(int i = 0; i<size/sizeof(int); i++)
		    {
		    	std::cout << data[i] << ", ";
		    }
		    	std::cout << std::endl;
	}
	void dispData(int i){ //Overloaded to only show short data report
			std::cout << "fileSize read - " << size << " kB, DataPoints - " << dataCount << std::endl;
	}

	int getDataValueAtIndex(int index)
	{
		if( (index < size) && (index > 0) )
		{
			return data[index];
		}
		else 
		{
			return 0;
		}
	}


}; 

#endif 
// BAIRCLAWDATAPLAYBACK_H






