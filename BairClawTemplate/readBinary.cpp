// reading an entire binary file
#include <iostream>
#include <fstream>
using namespace std;

int main () {
  streampos size;

  ifstream file ("data2Track.bin", ios::in|ios::binary|ios::ate);
  if (file.is_open())
  {
    size = file.tellg();
    int numberOfDataPoints = size/sizeof(int);
    int *data = new int[numberOfDataPoints];

    file.seekg (0, ios::beg);
    file.read ((char*)data, size);
    file.close();

    cout << "the entire file content is in memory" << endl;
   
   
    cout << "fileSize read - " << size << " kB, DataPoints - " << numberOfDataPoints << endl;
    for(int i = 0; i<numberOfDataPoints; i++)
    {
      cout << data[i] << ", ";
    }
    cout << endl;
    delete[] data;
  }
  else cout << "Unable to open file";
  return 0;
}