#include <iostream>
#include "json_handler.h"


JsonHandler::JsonHandler(string _json_string) {
    Json::Reader reader;
    reader.parse(_json_string.c_str(), m_json_value);

    switch(m_json_value["op"].asInt()){
        case 0: this->m_op_mode = CFG_2_POS;
                break;
        case 1: this->m_op_mode = POS_2_CFG;
                break;
        case 2: this->m_op_mode = PTP;
                break;
        case 3: this->m_op_mode = PTPSYNC;
				break;
		case 4: this->m_op_mode = LIN;
            break;
    }

    this->m_data = (m_json_value["data"]);
}


string JsonHandler::get_json_string(SixDPos* _pos)
{
    Json::Value value;
    Json::Value op(1);
    Json::Value data;
    data.append(*(_pos->serialize_to_json()));

    value["op"] = op;
    value["data"] = data;

    return value.toStyledString();
}


string JsonHandler::get_json_string(Configuration* _cfg)
{
    Json::Value value;
    Json::Value op(0);
    Json::Value data;
    data.append(*(_cfg->serialize_to_json()));

    value["op"] = op;
    value["data"] = data;

    return value.toStyledString();
}


string JsonHandler::get_json_string(vector<Configuration*>* _cfg)
{
    Json::Value value;
    Json::Value op(0);
    Json::Value data;
    for(Configuration* tmp_cfg : *_cfg)
    {
        data.append(*(tmp_cfg->serialize_to_json()));
    }

    value["op"] = op;
    value["data"] = data;

    return value.toStyledString();
}