#ifndef SDRI_CTRL2019_SIXDPOS_H
#define SDRI_CTRL2019_SIXDPOS_H

#include <json.h>

#include <cstddef>
#include <cstring>
#include <array>
#include <algorithm>
#include <vector>

#define NUM_DEGREES 6

using namespace std;

/**
 * This class is intended to store 6d position data
 */
class SixDPos {

private:
    vector<double> m_position;

public:

    /**
     *
     */
    enum elements {X=0, Y=1, Z=2, A=3, B=4, C=5};

    /**
     *
     */
	SixDPos() : m_position{ .0, .0, .0, .0, .0, .0} {};

	/**
	 *
	 * @param _position vector containing values for [x_pos, y_pos, z_pos, 1st_rot, 2nd_rot, 3rd_rot}
	 */
    SixDPos(vector<double> _position){
        this->set_position(_position);
    };

    /**
     *
     * @param _x    x position
     * @param _y    y position
     * @param _z    z position
     * @param _a    first rotation
     * @param _b    second rotation
     * @param _c    third rotation
     */
	SixDPos(double _x, double _y, double _z, double _a, double _b, double _c) : m_position{_x, _y, _z, _a, _b, _c} {};

	/**
	 *
	 * @param _json_value
	 */
	SixDPos(Json::Value _json_value);

	/**
	 *
	 */
    SixDPos(SixDPos& copy){
        this->set_position(copy.get_position());
    };

    /**
     *
     * @param copy
     */
    void operator=(SixDPos& copy){
        this->set_position(copy.get_position());
    }

    /**
     *
     * @param _position
     */
    void set_position(vector<double> _position){
        this->m_position = _position;
    };

    /**
     *
     * @return
     */
    vector<double>& get_position(){
        return this->m_position;
    }

    /**
     *
     * @param index
     * @return
     */
    double operator [] (size_t index){ return this->m_position[index];};

    /**
     *
     * @param index
     * @return
     */
    const double operator [] (size_t index) const { return this->m_position[index];};

    /**
     *
     * @return
     */
    const double get_X() const { return (*this)[X];};

    /**
     *
     * @return
     */
    const double get_Y() const { return (*this)[Y];};

    /**
     *
     * @return
     */
    const double get_Z() const { return (*this)[Z];};

    /**
     *
     * @return
     */
    const double get_A() const { return (*this)[A];};

    /**
     *
     * @return
     */
    const double get_B() const { return (*this)[B];};

    /**
     *
     * @return
     */
    const double get_C() const { return (*this)[C];};

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
