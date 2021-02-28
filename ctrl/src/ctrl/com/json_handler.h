#ifndef SDRI_CTRL2019_JSON_HANDLER_H
#define SDRI_CTRL2019_JSON_HANDLER_H

#include <string>
#include <json.h>
#include <SixDPos.h>
#include <Configuration.h>
#include "opmode_enum.h"
using namespace std;

/** \brief Json Handler for hte communication with the VREP GUI via remote api
 * This class handles the serialization and deserialization of controller results for the communication with the remote
 * api.
 *
 * The JSON string has the following format:
 * {
 *      "op": ->int operation mode {@see src/ctrl/com/opmode_enum.h}<-,
 *      "data": [->array containing the json strings from either the configurations or the positions<-]
 * }
 * In the following, an example of such a json string is presented. It represents an operation mode 1 with two
 * configuration.
 *
 * <code>
 * {
 *      "op": 1,
 *      "data": [
 *                  {
 *                      "j0": 0
 *                      "j1": 0.25
 *                      "j2": 0.5
 *                      "j3": 0.75
 *                      "j4": 1.0
 *                      "j5": 1.25
 *                  },
 *                  {
 *                      "j0": 1.0
 *                      "j1": 1.25
 *                      "j2": 1.5
 *                      "j3": 1.75
 *                      "j4": 2.0
 *                      "j5": 2.25
 *                  }
 *              ]
 * }
 * <\code>
 */
class JsonHandler {
public:
    /**
     * Constructor parsing a json string representation of the data initializes the handler
     * @param _json_string  json string in the given representation to be parsed
     */
    JsonHandler(string _json_string);

    /**
     * Returns the {@ref OpMode} from the parsed json string
     *
     * @return operation mode from the parsed json string
     */
    OpMode get_op_mode(){ return this->m_op_mode;   }

    /**
     * Returns the data array from the parsed json string
     *
     * @return Json::Value containing the data array from the parsed json string
     */
    Json::Value get_data(){ return this->m_data;    }

    /**
     * Returns the json string representation of the passed {@ref SixDPos}
     *
     * @param _pos  {@ref SixDPos} to be serialized
     * @return json string representation of the passed position
     */
    string get_json_string(SixDPos* _pos);

    /**
     * Returns the json string representation of the passed {@ref Configuration}
     *
     * @param _cfg {@ref Configuration} to be serialized
     * @return json string representation of the passed configuration
     */
    string get_json_string(Configuration* _cfg);

    /**
     * Returns the json string representation of the passed {@ref Configuration} vector
     *
     * @param _cfg vector containing a set of references of {@ref Configuration}
     * @return json string representation of the passed configurations
     */
    string get_json_string(vector<Configuration*>* _cfg);
private:
    /** json value parsed from the json string */
    Json::Value m_json_value;
    /** {@ref OpMode} parsed at the instantiation from the json string*/
    OpMode m_op_mode;
    /** json value containing the data array parsed from the json string*/
    Json::Value m_data;
};


#endif //SDRI_CTRL2019_JSON_HANDLER_H
