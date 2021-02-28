#include "Trajectory.h"
Trajectory::Trajectory()
{

}


Configuration* Trajectory::operator[](size_t index)
{
    return this->m_configs[index];
}


vector<Configuration*>* Trajectory::get_all_configuration()
{
    return &(this->m_configs);
}


Configuration* Trajectory::get_configuration(size_t index)
{
    return this->m_configs[index];
}


void Trajectory::operator=(Trajectory& copy)
{
    this->set_trajectory( *(copy.get_all_configuration()) );
}


void Trajectory::set_trajectory(const vector<Configuration*> _configurations)
{
    this->m_configs.clear();
    this->m_configs = _configurations;
}