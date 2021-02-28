#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;
#define PI 3.14159265
#define d_6 0.215  //Here the distance is 215mm
#define m 0.330   // the length of the m is 330mm
#define n 0.645   // the length of the n is 645mm
#define o 0.115   // the length of the o is 115mm
#define a 1.150   // the length of the a is 1150mm
#define b 1.220   // the length of the b is 1220mm



std::vector<double> find_theta1(double X_c, double Y_c);
//void find_theta2_theta3(double X_c, double Y_c, double Z_c, vector<double>& arr2, vector<double>& arr3);
void forward_calc(double Px_dash, double Py_dash, vector<double>& arr2, vector<double>& arr3);
void backward_calc(double Px_dash, double Py_dash, vector<double>& arr4, vector<double>& arr5);
std::vector<vector<double>> find_rotational_matrix_R0_3(double theta_1, double theta_2, double theta_3);
void find_theta4_theta5_theta6(double a_x, double a_y, double a_z, double n_z, double s_z, vector<double>& arr4, vector<double>& arr5, vector<double>& arr6);


vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position

    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!
    vector<Configuration*>* solutions = new vector<Configuration*>();
    

    //double new_X = 3.14099985e+02 /1000;
    //double new_Y = 3.13148732e+01 /1000;
    //double new_Z = 2.93478492e+03 /1000;
    
    double new_X = (*_pos)[0];
    double new_Y = (*_pos)[1];
    double new_Z = (*_pos)[2];
    double new_roll = (*_pos)[3];
    double new_pitch = (*_pos)[4];
    double new_yaw = (*_pos)[5];
    
    cout << "Position X:" << new_X << ", " << "Position Y:" << new_Y << ", " << "Position Z:" << new_Z << endl;
    cout << "Roll:" << new_roll << ", " << "Pitch:" << new_pitch << ", " << "Yaw:" << new_yaw << endl;
    
    // Les's find matrix R0_6 by using Raw Yaw and Pitch angle 
    //double R0_6[3][3];
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    
    r11 = cos(new_roll) * cos(new_pitch);
    r12 = (cos(new_roll) * sin(new_pitch) * sin(new_yaw)) - (sin(new_roll) * cos(new_yaw));
    r13 = (sin(new_roll) * sin(new_yaw)) + (cos(new_roll) * sin(new_pitch) * cos(new_yaw));
    r21 = sin(new_roll) * cos(new_pitch);
    r22 = (cos(new_roll) * cos(new_yaw)) + (sin(new_roll) * sin(new_pitch) * sin(new_yaw));
    r23 = (sin(new_roll) * sin(new_pitch) * cos(new_yaw)) - (cos(new_roll) * sin(new_yaw));
    r31 = -sin(new_pitch);
    r32 = cos(new_pitch) * sin(new_yaw);
    r33 = cos(new_pitch) * cos(new_yaw);
    
    std::vector<vector<double>> R0_6
    {
        {r11,r12,r13},
        {r21,r22,r23},
        {r31,r32,r33}
    };
    /*
    std::vector<vector<double>> R0_6
    {
        {2.58343520e-01, -9.28688786e-01, 2.66074735e-01},
        {9.39694139e-01, 3.05474825e-01, 1.53818259e-01},
        {-2.24128425e-01, 2.10290919e-01, 9.51600850e-01 }
    };
    
    r13 = R0_6[0][2];
    r23 = R0_6[1][2];
    r33 = R0_6[2][2];
    */

    //let's compute the wrist center position Oc [Xc, Yc, Zc]
    double X_c, Y_c, Z_c;
    X_c = new_X - (d_6 * r13);
    Y_c = new_Y - (d_6 * r23);
    Z_c = new_Z - (d_6 * r33);

    double d1 = sqrt(X_c * X_c + Y_c * Y_c);
    double Px_dash;
    double Py_dash = Z_c - n;

    //Compute the theta1 here
    std::vector<double> theta1_arr, theta2_arr, theta3_arr, theta4_arr, theta5_arr, theta6_arr;
    std::vector<double> theta1_f, theta2_f, theta3_f;
    theta1_arr = find_theta1(X_c, Y_c);
    //find_theta2_theta3(X_c, Y_c, Z_c, theta2_arr, theta3_arr);

    cout << "/n-----------------------------------/n" << endl;
    for (int i = 0; i < 1; i++)
    {

        if ((-175 < (theta1_arr[i]) && (theta1_arr[i]) < 175 && (theta1_arr[i]) != -90 && (theta1_arr[i]) != 90))
        {
            
            //from -175 to -91, from -89 to 89, from 91 to 175
            if (d1 > m)
            {
                Px_dash = d1 - m;
                forward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                for (int j = 0; j < theta2_arr.size(); j++)
                {
                    if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                    {
                        cout << "Theta1: " << theta1_arr[i] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                        theta1_f.push_back(theta1_arr[i]);
                        theta2_f.push_back(theta2_arr[j]);
                        theta3_f.push_back(theta3_arr[j]);

                    }
                }
                //if(theta2_arr.size()==1)
                  //  break;
                Px_dash = m + d1;
                if ((-185 < (theta1_arr[i+1]) && (theta1_arr[i+1]) < 185))
                {
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 1)
                        break;
                    int k;
                    if (theta2_arr.size() == 3)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() == 4)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+1] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+1]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }

                }
                if ((-185 < (theta1_arr[i + 2]) && (theta1_arr[i + 2]) < 185))
                {
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    int k;
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 4)
                        break;
                    if (theta2_arr.size() % 2 == 1)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() % 2 == 0)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+2] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+2]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
            }
            else if (d1 < m)
            {
                Px_dash = m - d1;
                backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                for (int j = 0; j < theta2_arr.size(); j++)
                {
                    if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                    {
                        cout << "Theta1: " << theta1_arr[i] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                        theta1_f.push_back(theta1_arr[i]);
                        theta2_f.push_back(theta2_arr[j]);
                        theta3_f.push_back(theta3_arr[j]);
                    }
                }

                Px_dash = m + d1;
                if ((-185 < (theta1_arr[i+1]) && (theta1_arr[i+1]) < 185))
                {
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 1)
                        break;
                    int k;
                    if (theta2_arr.size() == 3)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() == 4)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+1] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+1]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
                if ((-185 < (theta1_arr[i+2]) && (theta1_arr[i+2]) < 185))
                {
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 4)
                        break;
                    int k;
                    if (theta2_arr.size() % 2 == 1)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() % 2 == 0)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+2] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+2]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
            }

        }
        else if ((theta1_arr[i]) == -90)
        {
            if (Y_c > m)
            {
                d1 = Y_c;
                Px_dash = d1 - m;
                forward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                for (int j = 0; j < theta2_arr.size(); j++)
                {
                    if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                    {
                        cout << "Theta1: " << theta1_arr[i] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                        theta1_f.push_back(theta1_arr[i]);
                        theta2_f.push_back(theta2_arr[j]);
                        theta3_f.push_back(theta3_arr[j]);
                    }
                }

                if ((theta1_arr[i+1]) == 90)
                {
                    Px_dash = d1 + m;
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 1)
                        break;
                    int k;
                    if (theta2_arr.size() == 3)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() == 4)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i + 1] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i + 1]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }    
            }
            else if (Y_c < m)
            {
                d1 = Y_c;
                Px_dash = m - d1;
                backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                for (int j = 0; j < theta2_arr.size(); j++)
                {
                    if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                    {
                        cout << "Theta1: " << theta1_arr[i] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                        theta1_f.push_back(theta1_arr[i]);
                        theta2_f.push_back(theta2_arr[j]);
                        theta3_f.push_back(theta3_arr[j]);
                    }
                }

                if ((theta1_arr[i+1]) == 90)
                {
                    Px_dash = d1 + m;
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 1)
                        break;
                    int k;
                    if (theta2_arr.size() == 3)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() == 4)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i + 1] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i + 1]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
            }
        }

        else if ((theta1_arr[i]) == 90)
        {
            if (Y_c > m)
            {
                d1 = Y_c;
                Px_dash = d1 - m;
                forward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                for (int j = 0; j < theta2_arr.size(); j++)
                {
                    if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                    {
                        cout << "Theta1: " << theta1_arr[i] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                        theta1_f.push_back(theta1_arr[i]);
                        theta2_f.push_back(theta2_arr[j]);
                        theta3_f.push_back(theta3_arr[j]);
                    }
                }

                if ((theta1_arr[i + 1]) == -90)
                {
                    Px_dash = d1 + m;
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 1)
                        break;
                    int k;
                    if (theta2_arr.size() == 3)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() == 4)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i + 1] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i + 1]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
            }
            else if (Y_c < m)
            {
                d1 = Y_c;
                Px_dash = m - d1;
                backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                for (int j = 0; j < theta2_arr.size(); j++)
                {
                    if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                    {
                        cout << "Theta1: " << theta1_arr[i] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                        theta1_f.push_back(theta1_arr[i]);
                        theta2_f.push_back(theta2_arr[j]);
                        theta3_f.push_back(theta3_arr[j]);
                    }
                }

                if ((theta1_arr[i + 1]) == -90)
                {
                    Px_dash = d1 + m;
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 1)
                        break;
                    int k;
                    if (theta2_arr.size() == 3)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() == 4)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i + 1] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i + 1]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
            }

        }
    
 
        else if ((-185 < (theta1_arr[i]) && (theta1_arr[i]) < -175))
        {
            if (d1 > m)
            {
                Px_dash = d1 - m;
                forward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                for (int j = 0; j < theta2_arr.size(); j++)
                {
                    if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                    {
                        cout << "Theta1: " << theta1_arr[i] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                        theta1_f.push_back(theta1_arr[i]);
                        theta2_f.push_back(theta2_arr[j]);
                        theta3_f.push_back(theta3_arr[j]);
                    }
                }

                if ((-185 < (theta1_arr[i+1]) && (theta1_arr[i+1]) < 185))
                {
                    forward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 1)
                        break;
                    int k;
                    if (theta2_arr.size() == 3)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() == 4)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+1] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+1]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
                if ((-185 < (theta1_arr[i+2]) && (theta1_arr[i+2]) < 185))
                {
                    Px_dash = d1 + m;
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 4)
                        break;
                    int k;
                    if (theta2_arr.size() % 2 == 1)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() % 2 == 0)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+2] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+2]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }


            }
            else if (d1 < m)
            {
                Px_dash = m - d1;
                backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                for (int j = 0; j < theta2_arr.size(); j++)
                {
                    if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                    {
                        cout << "Theta1: " << theta1_arr[i] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                        theta1_f.push_back(theta1_arr[i]);
                        theta2_f.push_back(theta2_arr[j]);
                        theta3_f.push_back(theta3_arr[j]);
                    }
                }
                if ((-185 < (theta1_arr[i+1]) && (theta1_arr[i+1]) < 185))
                {
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 1)
                        break;
                    int k;
                    if (theta2_arr.size() == 3)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() == 4)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+1] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+1]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
                if ((-185 < (theta1_arr[i+2]) && (theta1_arr[i+2]) < 185))
                {
                    Px_dash = d1 + m;
                    forward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 4)
                        break;
                    int k;
                    if (theta2_arr.size() % 2 == 1)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() % 2 == 0)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+2] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+2]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }

            }

        }
        else if ((175 < (theta1_arr[i]) && (theta1_arr[i]) < 185))
        {
            if (d1 > m)
            {
                Px_dash = d1 - m;
                forward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                for (int j = 0; j < theta2_arr.size(); j++)
                {
                    if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                    {
                        cout << "Theta1: " << theta1_arr[i] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                        theta1_f.push_back(theta1_arr[i]);
                        theta2_f.push_back(theta2_arr[j]);
                        theta3_f.push_back(theta3_arr[j]);
                    }
                }
                if ((-185 < (theta1_arr[i+1]) && (theta1_arr[i+1]) < 185))
                {
                    forward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 1)
                        break;
                    int k;
                    if (theta2_arr.size() == 3)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() == 4)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+1] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+1]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
                if ((-185 < (theta1_arr[i+2]) && (theta1_arr[i+2]) < 185))
                {
                    Px_dash = d1 + m;
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 4)
                        break;
                    int k;
                    if (theta2_arr.size() % 2 == 1)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() % 2 == 0)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+2] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+2]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
                ;
            }
            else if (d1 < m)
            {
                Px_dash = m - d1;
                backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                for (int j = 0; j < theta2_arr.size(); j++)
                {
                    if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                    {
                        cout << "Theta1: " << theta1_arr[i] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                        theta1_f.push_back(theta1_arr[i]);
                        theta2_f.push_back(theta2_arr[j]);
                        theta3_f.push_back(theta3_arr[j]);
                    }
                }
                if ((-185 < (theta1_arr[i+1]) && (theta1_arr[i+1]) < 185))
                {
                    backward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 1)
                        break;
                    int k;
                    if (theta2_arr.size() == 3)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() == 4)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+1] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+1]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
                if ((-185 < (theta1_arr[i+2]) && (theta1_arr[i+2]) < 185))
                {
                    Px_dash = d1 + m;
                    forward_calc(Px_dash, Py_dash, theta2_arr, theta3_arr);
                    if (theta2_arr.size() == 2 || theta2_arr.size() == 4)
                        break;
                    int k;
                    if (theta2_arr.size() % 2 == 1)
                    {
                        k = theta2_arr.size() - 1;
                    }
                    else if (theta2_arr.size() % 2 == 0)
                    {
                        k = theta2_arr.size() - 2;
                    }
                    for (int j = k; j < theta2_arr.size(); j++)
                    {
                        if ((-140 < theta2_arr[j]) && (theta2_arr[j] < -5) && (-120 < theta3_arr[j]) && (theta3_arr[j] < 168))
                        {
                            cout << "Theta1: " << theta1_arr[i+2] << ", Theta2: " << theta2_arr[j] << ", Theta3: " << theta3_arr[j] << endl;
                            theta1_f.push_back(theta1_arr[i+2]);
                            theta2_f.push_back(theta2_arr[j]);
                            theta3_f.push_back(theta3_arr[j]);
                        }
                    }
                }
            }

        }



    }
    /*
    cout << "/n-----------------------------------/n" << endl;
    cout << "Size of theta1, theta2, theta3: " << theta1_f.size() << ", " << theta2_f.size() << ", " << theta3_f.size() << endl;

    cout << "/n-----------------------------------/n" << endl;
    for (int i = 0; i < theta1_f.size(); i++)
    {
        cout << "Theta1: " << theta1_f[i] << ", Theta2: " << theta2_f[i] << ", Theta3: " << theta3_f[i] << endl;
    }
    cout << "/n-----------------------------------/n" << endl;
    */
    
    std::vector<vector<double>> inv_R0_3, R3_6;
    inv_R0_3 =
    {
            {0,0,0},
            {0,0,0},
            {0,0,0}
    };
    
    double a_x, a_y, a_z, n_z, s_z;
    double th_1, th_2, th_3;
    
    
    //Compute the theta2 and theta3 here
    //std::vector<int>::size_type sz = theta1_arr.size();
    int k = 0;
    for (int j = 0; j < theta1_f.size(); j++)
    {
        // Here we are finding base rotational matrix R0_3
        

        inv_R0_3 = find_rotational_matrix_R0_3(theta1_f[j], theta2_f[j], theta3_f[j]);
        /*
        cout << "/n-----------------------------------/n" << endl;
        cout << "\nHere we are finding the transpose of base rotational matrix R0_3 \n" << endl;
        for (int i = 0; i < inv_R0_3.size(); i++)
        {

               for (int j = 0; j < inv_R0_3[i].size(); j++)
                    {
                        cout << inv_R0_3[i][j] << " ";
                    }
                    cout << endl;

        }
        cout << "/n-----------------------------------/n" << endl;
        */
        R3_6 =
        {
                {0,0,0},
                {0,0,0},
                {0,0,0}
        };
               
        
    // Here we are multiplying first two rotational matrix R0_3 and R0_6
        for (int l = 0; l < 3; l++)
        {
            for (int j = 0; j < 3; j++)
            {

               for (int k = 0; k < 3; k++)
               {
                    R3_6[l][j] += inv_R0_3[l][k] * R0_6[k][j];
               }
            }
        }
                /*
                cout << "\nHere we are multiplying two rotational matrix R0_3 and R0_6 and finding the matrix R3_6 \n" << endl;
                for (int i = 0; i < R3_6.size(); i++)
                {

                    for (int j = 0; j < R3_6[i].size(); j++)
                    {
                        cout << R3_6[i][j] << " ";
                    }
                    cout << endl;

                }
                */

                //Here are finding the angles theta 4, theta 5, theta 6
                
                
                a_x = R3_6[0][2];
                a_y = R3_6[1][2];
                a_z = R3_6[2][2];
                n_z = R3_6[2][0];
                s_z = R3_6[2][1];
                //cout << "The internal value of matrix R3_6:  " << a_x << ", " << a_y << ", " << a_z << ", " << n_z << ", " << s_z << endl;
                find_theta4_theta5_theta6(a_x, a_y, a_z, n_z, s_z, theta4_arr, theta5_arr, theta6_arr);

                cout << "Size of the theta_4_5_6 array: " << theta4_arr.size() << ", " << theta5_arr.size() << ", " << theta6_arr.size() << endl;
                cout << "\n---------Final output---------\n" << endl;
                

                for (int i =k; i < theta4_arr.size(); i++)
                {
                    //cout << "[ " << theta1_f[j] << " " << theta2_f[j] << " " << theta3_f[j] << " " << theta4_arr[i] << " " << theta5_arr[i] << " " << theta6_arr[i] << " ]" << endl;
                    cout << "[ " << theta1_f[j] * PI / 180 << " " << theta2_f[j] * PI / 180 << " " << theta3_f[j] * PI / 180 << " " << theta4_arr[i] * PI / 180 << " " << theta5_arr[i] * PI / 180 << " " << theta6_arr[i] * PI / 180 << " ]" << endl;
                    solutions->push_back(new Configuration({ theta1_f[j] * PI / 180,theta2_f[j] * PI / 180,theta3_f[j] * PI / 180,theta4_arr[i] * PI / 180,theta5_arr[i] * PI / 180,theta6_arr[i] * PI / 180 }));
                    k++;
                }
                cout << "/n-----------------------------------/n" << endl;
                
 
                
                
                    
               
            
        
        


    }
    
    theta4_arr.clear();
    theta5_arr.clear();
    theta6_arr.clear();

    /*
    
      solutions->push_back(new Configuration({0,0,1,0,0,0}));
      solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));
  */

    return solutions;
}

/*As we know from the Group one presentation if theta1 is between -175 and -175 or
    from 175 to 185 then we have two forward solutions and only one backward solution.
    In addition, when we have angle from -5 to 5, then we have one forward soluion
    and two backward solution
    */


std::vector<double> find_theta1(double X_c, double Y_c)
{
    //computing the taninverse using general mathmatics formula
    double theta1;
    theta1 = -atan2(Y_c, X_c) * 180 / PI;
    //cout << "Theta1: " << theta1 << endl;
    std::vector<double> arr;

    if ((-175 < (theta1) && (theta1) < 175)) {

        if (X_c > 0 && Y_c < 0) {

            arr.push_back(theta1);
            arr.push_back(theta1 + 180);
            arr.push_back(theta1 - 180);
        }
        else if (X_c < 0 && Y_c < 0) {


            arr.push_back(theta1);
            arr.push_back(theta1 + 180);
            arr.push_back(theta1 - 180);
        }
        else if (X_c < 0 && Y_c > 0) {

            arr.push_back(theta1);
            arr.push_back(theta1 + 180);
            arr.push_back(theta1 - 180);
        }
        else if (X_c > 0 && Y_c > 0) {
            arr.push_back(theta1);
            arr.push_back(theta1 + 180);
            arr.push_back(theta1 - 180);
        }
        else if (X_c == 0 && Y_c > 0) {
            arr.push_back(theta1);
            arr.push_back(theta1 + 180);

        }
        else if (X_c == 0 && Y_c < 0) {
            arr.push_back(theta1);
            arr.push_back(theta1 - 180);

        }
    }

    else if (-185 < (theta1) && (theta1) < -175) {
        double case1 = theta1;
        double case2 = theta1 + 180;
        double case3 = theta1 - 180;
        arr.push_back(case1);
        arr.push_back(case2);
        arr.push_back(case3);
    }
    else if (175 < (theta1) && (theta1) < 185) {
        double case1 = theta1;
        double case2 = theta1 + 180;
        double case3 = theta1 - 180;
        arr.push_back(case1);
        arr.push_back(case2);
        arr.push_back(case3);
    }



    return arr;
}


/*
void find_theta2_theta3(double X_c, double Y_c, double Z_c, vector<double>& arr2, vector<double>& arr3)
{
    double d_1, d_2, d_3, Px_dash, Py_dash, theta2, theta3, beta1, alpha1, alpha2;
    d_1 = sqrt(X_c * X_c + Y_c * Y_c);
    d_2 = sqrt(b * b + o * o);
    if (d_1 > m)
    {
        Px_dash = d_1 - m;
        Py_dash = Z_c - n;
        d_3 = sqrt(Px_dash * Px_dash + Py_dash * Py_dash);
        beta1 = acos(((d_3 * d_3) - (a * a) - (d_2 * d_2)) / (-2 * a * d_2)) * 180 / PI;
        alpha1 = asin(sin(beta1 * PI / 180) * (d_2 / d_3)) * 180 / PI;
        alpha2 = asin(Py_dash / d_3) * 180 / PI;
        //forward elbow down

        theta2 = -(alpha2 - alpha1);
        theta3 = beta1 - (asin(b / d_2) * 180 / PI) - 90;
        arr2.push_back(theta2);
        arr3.push_back(theta3);

        // forward elbow up
        theta2 = -(alpha1 + alpha2);
        theta3 = 360 - beta1 - (asin(b / d_2) * 180 / PI) - 90;
        arr2.push_back(theta2);
        arr3.push_back(theta3);


    }
    else if (d_1 < m)
    {
        Px_dash = m - d_1;
        Py_dash = Z_c - n;
        d_3 = sqrt(Px_dash * Px_dash + Py_dash * Py_dash);
        beta1 = acos(((d_3 * d_3) - (a * a) - (d_2 * d_2)) / (-2 * a * d_2)) * 180 / PI;
        alpha1 = asin(sin(beta1 * PI / 180) * (d_2 / d_3)) * 180 / PI;
        alpha2 = asin(Py_dash / d_3) * 180 / PI;
        //bacward elbow down

        theta2 = (alpha2 - alpha1) - 180;

        theta3 = -(90 - (beta1 - (asin(b / d_2) * 180 / PI)));
        arr2.push_back(theta2);
        arr3.push_back(theta3);

        //backward elbow down

        theta2 = (alpha1 + alpha2) - 180;
        theta3 = 270 - beta1 - (asin(b / d_2) * 180 / PI);
        arr2.push_back(theta2);
        arr3.push_back(theta3);
    }
}
*/

void forward_calc(double Px_dash, double Py_dash, vector<double>& arr2, vector<double>& arr3)
{
    double d_2, d_3, theta2, theta3, beta1, alpha1, alpha2;
    d_2 = sqrt(b * b + o * o);
    d_3 = sqrt(Px_dash * Px_dash + Py_dash * Py_dash);
    beta1 = acos(((d_3 * d_3) - (a * a) - (d_2 * d_2)) / (-2 * a * d_2)) * 180 / PI;
    alpha1 = asin(sin(beta1 * PI / 180) * (d_2 / d_3)) * 180 / PI;
    alpha2 = asin(Py_dash / d_3) * 180 / PI;

    //forward elbow down

    theta2 = -(alpha2 - alpha1);
    theta3 = beta1 - (asin(b / d_2) * 180 / PI) - 90;
    if ((-140 < theta2) && (theta2 < -5) && (-120 < theta3) && (theta3 < 168))
    {
        arr2.push_back(theta2);
        arr3.push_back(theta3);
    }


    // forward elbow up
    theta2 = -(alpha1 + alpha2);
    theta3 = 360 - beta1 - (asin(b / d_2) * 180 / PI) - 90;
    if ((-140 < theta2) && (theta2 < -5) && (-120 < theta3) && (theta3 < 168))
    {
        arr2.push_back(theta2);
        arr3.push_back(theta3);
    }

}

void backward_calc(double Px_dash, double Py_dash, vector<double>& arr4, vector<double>& arr5)
{
    double d_2, d_3, theta2, theta3, beta1, alpha1, alpha2;
    d_2 = sqrt(b * b + o * o);
    d_3 = sqrt(Px_dash * Px_dash + Py_dash * Py_dash);
    beta1 = acos(((d_3 * d_3) - (a * a) - (d_2 * d_2)) / (-2 * a * d_2)) * 180 / PI;
    alpha1 = asin(sin(beta1 * PI / 180) * (d_2 / d_3)) * 180 / PI;
    alpha2 = asin(Py_dash / d_3) * 180 / PI;
    //bacward elbow down

    theta2 = (alpha2 - alpha1) - 180;
    theta3 = 270 - beta1 - (asin(b / d_2) * 180 / PI);
    if ((-140 < theta2) && (theta2 < -5) && (-120 < theta3) && (theta3 < 168))
    {
        arr4.push_back(theta2);
        arr5.push_back(theta3);
    }

    //backward elbow up

    theta2 = (alpha1 + alpha2) - 180;
    theta3 = -(90 - (beta1 - (asin(b / d_2) * 180 / PI)));
    if ((-140 < theta2) && (theta2 < -5) && (-120 < theta3) && (theta3 < 168))
    {
        arr4.push_back(theta2);
        arr5.push_back(theta3);
    }

}

std::vector<vector<double>> find_rotational_matrix_R0_3(double theta_1, double theta_2, double theta_3)
{
    double nx, ny, nz, sx, sy, sz, ax, ay, az, th, al;
    double alpha[4] = { 180, 90, 0, 90 };
    double theta[4] = { 0, theta_1, theta_2, theta_3 - 90 };
    std::vector<vector<double>> R0_1, R1_2, R2_3, R3_4, R0_2, R2_4, R0_3, new_R0_3;

    R0_2 =
    {
            {0,0,0},
            {0,0,0},
            {0,0,0}
    };

    R2_4 =
    {
            {0,0,0},
            {0,0,0},
            {0,0,0}
    };

    R0_3 =
    {
            {0,0,0},
            {0,0,0},
            {0,0,0}
    };

    new_R0_3 =
    {
            {0,0,0},
            {0,0,0},
            {0,0,0}
    };

    //Here we are finding the rotation matrix R0_1
    {
        th = theta[0];
        al = alpha[0];
        nx = cos(th * PI / 180);
        sx = -sin(th * PI / 180) * cos(al * PI / 180);
        ax = sin(th * PI / 180) * sin(al * PI / 180);
        ny = sin(th * PI / 180);
        sy = cos(th * PI / 180) * cos(al * PI / 180);
        ay = -cos(th * PI / 180) * sin(al * PI / 180);
        nz = 0;
        sz = sin(al * PI / 180);
        az = cos(al * PI / 180);
        R0_1 =
        {
            {nx,sx,ax},
            {ny,sy,ay},
            {nz,sz,az}
        };
    }

    //Here we are finding the rotation matrix R1_2
    {
        th = theta[1];
        al = alpha[1];
        nx = cos(th * PI / 180);
        sx = -sin(th * PI / 180) * cos(al * PI / 180);
        ax = sin(th * PI / 180) * sin(al * PI / 180);
        ny = sin(th * PI / 180);
        sy = cos(th * PI / 180) * cos(al * PI / 180);
        ay = -cos(th * PI / 180) * sin(al * PI / 180);
        nz = 0;
        sz = sin(al * PI / 180);
        az = cos(al * PI / 180);
        R1_2 =
        {
            {nx,sx,ax},
            {ny,sy,ay},
            {nz,sz,az}
        };
    }

    //Here we are finding the rotation matrix R2_3
    {
        th = theta[2];
        al = alpha[2];
        nx = cos(th * PI / 180);
        sx = -sin(th * PI / 180) * cos(al * PI / 180);
        ax = sin(th * PI / 180) * sin(al * PI / 180);
        ny = sin(th * PI / 180);
        sy = cos(th * PI / 180) * cos(al * PI / 180);
        ay = -cos(th * PI / 180) * sin(al * PI / 180);
        nz = 0;
        sz = sin(al * PI / 180);
        az = cos(al * PI / 180);;
        R2_3 =
        {
            {nx,sx,ax},
            {ny,sy,ay},
            {nz,sz,az}
        };
    }

    //Here we are finding the rotation matrix R3_4
    {
        th = theta[3];
        al = alpha[3];
        nx = cos(th * PI / 180);
        sx = -sin(th * PI / 180) * cos(al * PI / 180);
        ax = sin(th * PI / 180) * sin(al * PI / 180);
        ny = sin(th * PI / 180);
        sy = cos(th * PI / 180) * cos(al * PI / 180);
        ay = -cos(th * PI / 180) * sin(al * PI / 180);
        nz = 0;
        sz = sin(al * PI / 180);
        az = cos(al * PI / 180);
        R3_4 =
        {
            {nx,sx,ax},
            {ny,sy,ay},
            {nz,sz,az}
        };
    }

    // Here we are multiplying first two rotational matrix R0_1 and R1_2
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {

            for (int k = 0; k < 3; k++)
            {
                R0_2[i][j] += R0_1[i][k] * R1_2[k][j];
            }
        }
    }

    // Here we are multiplying last two rotational matrix R2_3 and R3_4
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {

            for (int k = 0; k < 3; k++)
            {
                R2_4[i][j] += R2_3[i][k] * R3_4[k][j];
            }
        }
    }

    // Here we are multiplying the result of multiplication and we will get the R0_3
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {

            for (int k = 0; k < 3; k++)
            {
                R0_3[i][j] += R0_2[i][k] * R2_4[k][j];
            }
        }
    }
    /*
    cout << "\nHere we are finding the base rotational matrix R0_3 \n" << endl;
    for (int i = 0; i < R0_3.size(); i++)
    {

        for (int j = 0; j < R0_3[i].size(); j++)
        {
            cout << R0_3[i][j] << " ";
        }
        cout << endl;

    }
    */
    //Here we are going to find the transpose matrix using basic mathmatics formula
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            new_R0_3[i][j] = R0_3[j][i];
        }
    }


    return new_R0_3;

}

void find_theta4_theta5_theta6(double a_x, double a_y, double a_z, double n_z, double s_z, vector<double>& arr4, vector<double>& arr5, vector<double>& arr6)
{
    double theta_4, theta_5, theta_6;

    //here we are conmputing theta4
    theta_4 = atan2(-a_y, -a_x) * 180 / PI;
    if ((theta_4 - 360) > -350)
    {
        arr4.push_back(theta_4);
        arr4.push_back(theta_4);
        arr4.push_back(theta_4 - 360);
        arr4.push_back(theta_4 - 360);
    }
    else if ((theta_4 + 360) < 350)
    {
        arr4.push_back(theta_4);
        arr4.push_back(theta_4);
        arr4.push_back(theta_4 + 360);
        arr4.push_back(theta_4 + 360);
    }

    // here we are computing theta5 and theta6
    theta_5 = atan2(sqrt(1 - (a_z * a_z)), -a_z) * 180 / PI;
    theta_6 = atan2(s_z, n_z) * 180 / PI;
    if ((theta_6 - 360) > -350)
    {
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr6.push_back(theta_6);
        arr6.push_back(theta_6 - 360);
        arr6.push_back(theta_6);
        arr6.push_back(theta_6 - 360);
        
        

    }
    else if ((theta_6 + 360) < 350)
    {
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr6.push_back(theta_6);
        arr6.push_back(theta_6 + 360);
        arr6.push_back(theta_6);
        arr6.push_back(theta_6 + 360);
    }
    
    theta_5 = -theta_5;
    if (theta_4 > 0)
    {
        theta_4 = theta_4 - 180;
    }
    else
    {
        theta_4 = theta_4 + 180;
    }

    if ((theta_4 - 360) > -350)
    {
        arr4.push_back(theta_4);
        arr4.push_back(theta_4);
        arr4.push_back(theta_4 - 360);
        arr4.push_back(theta_4 - 360);
    }
    else if ((theta_4 + 360) < 350)
    {
        arr4.push_back(theta_4);
        arr4.push_back(theta_4);
        arr4.push_back(theta_4 + 360);
        arr4.push_back(theta_4 + 360);
    }

    if (theta_6 > 0)
    {
        theta_6 = theta_6 - 180;
    }
    else
    {
        theta_6 = theta_6 + 180;
    }

    if ((theta_6 - 360) > -350)
    {
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr6.push_back(theta_6);
        arr6.push_back(theta_6 - 360);
        arr6.push_back(theta_6);
        arr6.push_back(theta_6 - 360);

    }
    else if ((theta_6 + 360) < 350)
    {
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr6.push_back(theta_6);
        arr6.push_back(theta_6 + 360);
        arr6.push_back(theta_6);
        arr6.push_back(theta_6 + 360);
    }
    /*
    cout << "Size of the theta_4_5_6 array: " << arr4.size() << ", " << arr5.size() << ", " << arr6.size() << endl;
    cout << "\n---------Final output---------\n" << endl;
    for (int i = arr4.size()-8; i < arr4.size(); i++) {

        cout << "[ " << arr4[i] << " " << arr5[i] << " " << arr6[i] << " ]" << endl;
    }
    */
}

