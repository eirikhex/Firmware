#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <matrix/math.hpp>
#include <matrix/Matrix.hpp>
#include <iostream>


#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace std;
using namespace matrix;

extern "C" __EXPORT int test_app_main(int argc, char *argv[]);


class test_app
{
public:

private:
    float tmp;
    float u[5];
    float input[5];
    float m[5][6];
};

//methods/functions defined in this scope
void testFunction();



void testFunction()
{
    cout << "Entered testFunction function-call." << endl;
}

int test_app_main(int argc, char *argv[])
{
    PX4_INFO("Hello. You just ran your .cpp-file!");
    cout << "_main()." << endl;

    testFunction();

    // @Input: input
    // @output: thruster force vector
    // ex: input = [-0.25 0 1 0 0]; %Desired force in X Y Z K N

    float input[] = {1, 0, 1, 0, 0};
    float m[5][6];

    //Matrix3f m;
    //m.setZero();
    //m.zero();
    m[0][0] = 1;
    m[0][1] = -1;
    m[0][2] = 0;
    m[0][3] = 0;
    m[0][4] = -1;

    m[1][0] = 1;
    m[1][1] = 1;
    m[1][2] = 0;
    m[1][3] = 0;
    m[1][4] = 1;

    m[2][0] = 1;
    m[2][1] = 1;
    m[2][2] = 0;
    m[2][3] = 0;
    m[2][4] = -1;

    m[3][0] = 1;
    m[3][1] = -1;
    m[3][2] = 0;
    m[3][3] = 0;
    m[3][4] = 1;

    m[4][0] = 0;
    m[4][1] = 0;
    m[4][2] = 1;
    m[4][3] = 1;
    m[4][4] = 0;

    m[5][0] = 0;
    m[5][1] = 0;
    m[5][2] = 1;
    m[5][3] = 1;
    m[5][4] = 0;

    float u[] = {0, 0, 0, 0, 0, 0};
    float tmp;

    //Multiplication: u = inv(K)*pinv(T)*tauScaler*tau;
    //Remark: tauScaler scales so that
    for( int i = 0; i < 6; i = i + 1 ) {
        tmp = 0;
        for( int j = 0; j < 5; j = j + 1 ) {

            tmp = tmp + m[i][j]*input[j];

        }
        u[i] = tmp;
        cout << "value of u[ " << i << "] = " << u[1] << endl;
       }

    //Saturation
    for( int k = 0; k < 6; k = k + 1 ) {
        if(u[k] < -1) {
            cout << "Correcting value for u[ " << k << "] = " << u[k] << " to -1." << endl;
            u[k] = -1;
        }
        if(u[k] > 1) {
            cout << "Correcting value for u[ " << k << "] = " << u[k] << " to 1." << endl;
            u[k] = 1;
        }

    }

    /*Examples
    * @input = [0.5, 0, 0, 0, 0]  gives @output = [0.5, 0.5, 0.5, 0.5, 0, 0]
    * @input = [0.24, 0, 1, 0, 0] gives @output = [0.24, 0.24, 0.24, 0.24, 1, 1]
    *
    *
    *
    */

    return 0;
    return OK;
}

