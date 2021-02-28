#include <iostream>
#include "ctrl/sdir_ctrl.h"
#include "ctrl/com/json_handler.h"
#include <string>
#include <stdio.h>
#include <chrono>
#include <thread>


extern "C" {
#include "ctrl/com/remoteApi/extApi.h"
}

/** \brief main entry point of the sdir project
 *  This is the main class of the sdir programming project. It represents the interface to the extern context. In particular
 *  to the VREP environment. It reads the json string form the gui and interprets the respective operations.
 *  The four main operations are:
 *   - POS_2_CFG: In this mode the receiving json string contains a {@ref SixDPos}. From this, an inverse kinematic must
 *                be computed and the computed {@ref Configuration} is returned.
 *
 *   - CFG_2_POS In this mode the receiving json string contains a {@ref Configuration}. From this, the direct kinematic
 *               must be computed and the computed {@ref SixDPos} is returned.
 *
 *   - PTP In this mode the json string contains a start and and a target configuration. From this a {@ref PTP} movement
 *         is computed and the resulting {@ref Trajectory} is used to move the robot manipulator. This is done by calling
 *         the VREP remote api function runConfig. By using the sleep command the update interval between the configurations
 *         is handled.
 *
 *   - LIN In this mode the json string contains a start and and a target configuration. From this a {@ref LIN} movement
 *         is computed and the resulting {@ref Trajectory} is used to move the robot manipulator. This is done by calling
 *         the VREP remote api function runConfig. By using the sleep command the update interval between the configurations
 *         is handled.
 *
 * As long as you do not change the communication between GUI and controller this class must not be changed.
 * The only thing you should take care of is the update rate for sending configuration values to the robot. Thsi should
 * be synchronized with VREP and your trajectory time resolution.
 */

using namespace std;

simxInt* jh = new simxInt[6];

simxInt Initial()
{
    string Joint;
    simxInt *check = new simxInt[1];
    int portNb = 19997;
    simxInt clientID = -1;
    simxFinish(-1);

    clientID = simxStart("127.0.0.1", portNb, true, true, 5000, 5);
//    clientID = simxStart("141.44.29.96", portNb, true, true, 5000, 5);

    if (clientID > -1)
    {
        //Starten der Simulation
        simxStartSimulation(clientID, simx_opmode_oneshot);
		cout << "Simulation started" << endl;

        for (int i = 0; i < 6; i++)
        {
            //Abfrage der einzelnen Joint Handles des Manipulators
            Joint = "KR120_2700_2_joint" + to_string(i + 1);
            const char* c = Joint.c_str();
            simxGetObjectHandle(clientID, (const simxChar *)c, check, simx_opmode_oneshot_wait);
            jh[i] = check[0];
        }
    }
    else
    {
        printf("Connection Fail");
        getchar();
        extApi_sleepMs(10);
    }

    return clientID;
}

int main() {
    cout << "This is the entry point of the SDIR programming project" << endl;
    SdirCtrl ctrl;
    float c[6];
    simxInt ID = Initial();
    simxUChar* value = new simxUChar;
    simxInt length = -1;

    setvbuf(stdout, nullptr, _IONBF, 0);

    simxGetStringSignal(ID, "callsignal", &value, &length, simx_opmode_streaming);

    /*
     * While VREP remote connection is available
     */
    while (simxGetConnectionId(ID) != -1 ) {
        // read data from vrep
        simxGetStringSignal(ID, "callsignal", &value, &length, simx_opmode_blocking);

        // if data via signal "callsignal" has been received
        if(length > 0)
        {
			// cast vrep data to string
            string t = string(reinterpret_cast<const char*>(value), length);
//            cout << t << endl;
            // deserialize the json input
            JsonHandler jsonHandler(t);

            /*
             * compute a configuration from a position
             * inverse kinematics implementation
             */
            if(jsonHandler.get_op_mode() == OpMode::POS_2_CFG){
                SixDPos pos(jsonHandler.get_data()[0]);
//                cout << jsonHandler.get_json_string(&pos) << endl;
                vector<Configuration*>* result_cfg = ctrl.get_config_from_pos(&pos);
                string json_return_string = jsonHandler.get_json_string(result_cfg);
//                cout << json_return_string << endl;
                simxSetStringSignal(ID, "returnsignal",
                                    reinterpret_cast<const simxUChar *>(json_return_string.c_str()), json_return_string.length(), simx_opmode_oneshot);
            }

            /*
             * compute a position from a configuration
             * Direct kinematics implemntation
             */
            if(jsonHandler.get_op_mode() == OpMode::CFG_2_POS){
                Configuration cfg(jsonHandler.get_data()[0]);
//                cout << jsonHandler.get_json_string(&cfg) << endl;
                SixDPos* return_pos = ctrl.get_pos_from_config(&cfg);
                string json_return_string = jsonHandler.get_json_string(return_pos);
//                cout << json_return_string << endl;
                simxSetStringSignal(ID, "returnsignal",
                                    reinterpret_cast<const simxUChar *>(json_return_string.c_str()), json_return_string.length(), simx_opmode_oneshot);
            }

            /*
             * Compute a PTP trajectory and move the robot asynchronous in VREP
             */
            if(jsonHandler.get_op_mode() == OpMode::PTP){
                Configuration start_cfg((jsonHandler.get_data())[0]);
                Configuration end_cfg((jsonHandler.get_data())[1]);
//                cout << "Path Configuration" << endl;
//                cout << "start config: " << start_cfg [0] << ", " << start_cfg [1] << ", " << start_cfg [2] << ", " << start_cfg [3] << ", " << start_cfg [4] << ", " << start_cfg [5] << endl;
//                cout << "end config: " << end_cfg [0] << ", " << end_cfg [1] << ", " << end_cfg [2] << ", " << end_cfg [3] << ", " << end_cfg [4] << ", " << end_cfg [5] << endl;

                Trajectory* trajectory = ctrl.move_robot_ptp(&start_cfg, &end_cfg);
                for (Configuration* cur_cfg : *(trajectory->get_all_configuration())) {
                    c[0] = (*cur_cfg)[0];
                    c[1] = (*cur_cfg)[1];
                    c[2] = (*cur_cfg)[2];
                    c[3] = (*cur_cfg)[3];
                    c[4] = (*cur_cfg)[4];
                    c[5] = (*cur_cfg)[5];
                    simxCallScriptFunction(ID, "KR120_2700_2", sim_scripttype_childscript, "runConfig", 0, NULL, 6, c, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
                    // synchronize with vrep simulation environment
                    this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }

			/*
			 * Compute a PTP trajectory and move the robot synchronous in VREP
			 */
			if (jsonHandler.get_op_mode() == OpMode::PTPSYNC) {
				Configuration start_cfg((jsonHandler.get_data())[0]);
				Configuration end_cfg((jsonHandler.get_data())[1]);
				//                cout << "Path Configuration" << endl;
				//                cout << "start config: " << start_cfg [0] << ", " << start_cfg [1] << ", " << start_cfg [2] << ", " << start_cfg [3] << ", " << start_cfg [4] << ", " << start_cfg [5] << endl;
				//                cout << "end config: " << end_cfg [0] << ", " << end_cfg [1] << ", " << end_cfg [2] << ", " << end_cfg [3] << ", " << end_cfg [4] << ", " << end_cfg [5] << endl;

				Trajectory* trajectory = ctrl.move_robot_ptp(&start_cfg, &end_cfg);
				for (Configuration* cur_cfg : *(trajectory->get_all_configuration())) {
					c[0] = (*cur_cfg)[0];
					c[1] = (*cur_cfg)[1];
					c[2] = (*cur_cfg)[2];
					c[3] = (*cur_cfg)[3];
					c[4] = (*cur_cfg)[4];
					c[5] = (*cur_cfg)[5];
					simxCallScriptFunction(ID, "KR120_2700_2", sim_scripttype_childscript, "runConfig", 0, NULL, 6, c, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
					// synchronize with vrep simulation environment
					this_thread::sleep_for(std::chrono::milliseconds(50));
				}
			}

            /*
             * Compute a LIN trajectory and move the robot in VREP
             */
            if(jsonHandler.get_op_mode() == OpMode::LIN){
                Configuration start_cfg((jsonHandler.get_data())[0]);
                Configuration end_cfg((jsonHandler.get_data())[1]);
//                cout << "Path Configuration" << endl;
//                cout << "start config: " << start_cfg [0] << ", " << start_cfg [1] << ", " << start_cfg [2] << ", " << start_cfg [3] << ", " << start_cfg [4] << ", " << start_cfg [5] << endl;
//                cout << "end config: " << end_cfg [0] << ", " << end_cfg [1] << ", " << end_cfg [2] << ", " << end_cfg [3] << ", " << end_cfg [4] << ", " << end_cfg [5] << endl;

                Trajectory* trajectory = ctrl.move_robot_lin(&start_cfg, &end_cfg);
                for (Configuration* cur_cfg : *(trajectory->get_all_configuration())) {
                    c[0] = (*cur_cfg)[0];
                    c[1] = (*cur_cfg)[1];
                    c[2] = (*cur_cfg)[2];
                    c[3] = (*cur_cfg)[3];
                    c[4] = (*cur_cfg)[4];
                    c[5] = (*cur_cfg)[5];
                    simxCallScriptFunction(ID, "KR120_2700_2", sim_scripttype_childscript, "runConfig", 0, NULL, 6, c, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
                    // synchronize with vrep simulation environment
                    this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }

            simxClearStringSignal(ID, "callsignal", simx_opmode_blocking);
            length = 0;
        }
    }
    simxFinish(ID);

    return 0;
}