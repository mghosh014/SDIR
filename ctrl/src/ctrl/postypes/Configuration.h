#ifndef SDRI_CTRL2019_CONFIGURATION_H
#define SDRI_CTRL2019_CONFIGURATION_H


#include <cstddef>
#include <initializer_list>
#include <array>
#include <cstring>
#include "json.h"


#define NUM_JOINTS 6
/**
 * This class can be used to store data of a robot configuration
 */
class Configuration {

private:

	std::array<double, NUM_JOINTS> joints {};

public:
    /**
     *
     */
	Configuration();

	/**
	 *
	 * @param _joints
	 */
	Configuration(std::array<double,NUM_JOINTS> _joints);

	/**
	 *
	 * @param _jv
	 */
    Configuration(Json::Value _jv);

    /**
     *
     * @param copy
     */
	Configuration(const Configuration& copy);

	/**
	 *
	 * @param _joints
	 */
	void set_configuration(const std::array<double,NUM_JOINTS> _joints);

	/**
	 *
	 * @param index
	 * @return
	 */
	double& operator[](size_t index);

	/**
	 *
	 * @param index
	 * @return
	 */
	const double operator[](size_t index) const ;

	/**
	 *
	 */
	std::array<double,NUM_JOINTS>& get_configuration();

	/**
	 *
	 * @return
	 */
	const std::array<double,NUM_JOINTS>& get_configuration() const ;

	/**
	 *
	 * @param copy
	 */
	void operator=(const Configuration& copy);

	/**
	 *
	 * @return
	 */
	Json::Value* serialize_to_json();

	/**
	 *
	 * @param json_value
	 */
    void deserialize_from_json(Json::Value json_value);

};
#endif
