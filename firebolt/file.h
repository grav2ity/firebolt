#ifndef FILE_H
#define FILE_H

#include <fstream>


struct Node;

class SceneReader {

public:

  void readScene(char*);

private:

  void readMesh();
  void readNode(Node*);

  void createObjects(Node*);


  std::ifstream file;

  unsigned int refNumber;
  unsigned int meshCount;
  unsigned int animationCount;

};

extern SceneReader sceneReader;

#endif
