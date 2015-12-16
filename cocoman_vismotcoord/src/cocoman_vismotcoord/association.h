#ifndef ASSOCIATION_H
#define ASSOCIATION_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "scene.h"
#include "model.h"

using namespace std;
using namespace Eigen;

struct AssEntry{
    int indexScene;
    Vector3i indexModel;

    float dist;

};

class Association
{
public:
    Association();
    void init(int dimScene, int dimModel);
    void set(Scene scene, Model model);

private:
    AssEntry *assEntry;
};

#endif // ASSOCIATION_H
