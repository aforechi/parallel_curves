#include "read_pgm.h"

#include <cstdlib>
#include <climits>
#include <cstring>
#include <fstream>
#include <iostream>

#define BUF_SIZE 256

class pnm_error {};

void pnm_read(std::ifstream &file, char *buf) 
{
  char doc[BUF_SIZE];
  char c;
  
  file >> c;
  while (c == '#') {
    file.getline(doc, BUF_SIZE);
    file >> c;
  }
  file.putback(c);
  
  file.width(BUF_SIZE);
  file >> buf;
  file.ignore();
}

u_char* readPGM(const char *name, int& width, int& height) 
{
  char buf[BUF_SIZE];
  
  // read header
  std::ifstream file(name, std::ios::in | std::ios::binary);
  pnm_read(file, buf);
  if (strncmp(buf, "P5", 2)) {
    std::cout << "ERROR: Could not read file " << name << std::endl;
    throw pnm_error();
  }

  pnm_read(file, buf);
  width = atoi(buf);
  pnm_read(file, buf);
  height = atoi(buf);

  pnm_read(file, buf);
  if (atoi(buf) > UCHAR_MAX) {
    std::cout << "ERROR: Could not read file " << name << std::endl;
    throw pnm_error();
  }

  // read data
  u_char* cmap = new u_char[width * height];
  file.read((char *)cmap, width * height * sizeof(u_char));

  return cmap;
}
