#ifndef SDRI_CTRL2019_TRAJECTORY_H
#define SDRI_CTRL2019_TRAJECTORY_H

#include <vector>
#include "Configuration.h"

using namespace std;

class Trajectory {
private:
    vector<Configuration*> m_configs;
public:
    Trajectory();

    Configuration* operator[](size_t index);


    Configuration* get_configuration(size_t index);

    vector<Configuration*>* get_all_configuration();

    void operator=(Trajectory& copy);


    void add_configuration(Configuration* config);

    void set_trajectory(vector<Configuration*> _trajectory);

};


#endif //SDRI_CTRL2019_TRAJECTORY_H
