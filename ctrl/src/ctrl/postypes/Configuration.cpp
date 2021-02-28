#include "Configuration.h"

#include <cstring>
#include <iostream>

Json::Value* Configuration::serialize_to_json()
{
	Json::Value* json_pose = new Json::Value();
	(*json_pose)["j0"] = this->get_configuration()[0];
	(*json_pose)["j1"] = this->get_configuration()[1];
	(*json_pose)["j2"] = this->get_configuration()[2];
	(*json_pose)["j3"] = this->get_configuration()[3];
	(*json_pose)["j4"] = this->get_configuration()[4];
	(*json_pose)["j5"] = this->get_configuration()[5];

	return json_pose;
}


void Configuration::deserialize_from_json(Json::Value _jv){
    this->set_configuration({_jv["j0"].asDouble(), _jv["j1"].asDouble(), _jv["j2"].asDouble(), _jv["j3"].asDouble(), _jv["j4"].asDouble(), _jv["j5"].asDouble()});
}


Configuration::Configuration(Json::Value _jv){
    this->deserialize_from_json(_jv);
}


Configuration::Configuration() : joints{0,0,0,0,0,0}
{

}

Configuration::Configuration(std::array<double,NUM_JOINTS> _joints) : Configuration()
{
    this->set_configuration(_joints);
}

Configuration::Configuration(const Configuration& copy) : Configuration()
{
    this->set_configuration(copy.get_configuration());
}

void Configuration::set_configuration(const std::array<double,NUM_JOINTS> _joints)
{
    memcpy(this->joints.data(), _joints.data(), sizeof(double) * NUM_JOINTS);
}

/* Inline functions */

double& Configuration::operator[](size_t index)
{
    return this->joints[index];
}

const double Configuration::operator[](size_t index) const
{
    return this->joints[index];
}

std::array<double,NUM_JOINTS>& Configuration::get_configuration()
{
    return joints;
}

const std::array<double,NUM_JOINTS>& Configuration::get_configuration() const
{
    return joints;
}

void Configuration::operator=(const Configuration& copy)
{
    this->set_configuration( copy.get_configuration() );
}
