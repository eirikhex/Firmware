#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <matrix/math.hpp>
#include <matrix/Matrix.hpp>
#include <iostream>


using namespace std;
using namespace matrix;

//extern "C" __EXPORT int main(int argc, char *argv[]);


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
void testFunction(float input[]);



void testFunction(float input[])
{
    // @Input: Desired force in X,Y,Z,ROLL,YAW
    // @output: thruster force vector
    // ex: input = {-0.25, 0, 1, 0, 0};
    //

    //Defining the matrix
    float m[5][6];
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

    float u[] = {0, 0, 0, 0, 0, 0}; //Thrust force vector
    float tmp;

    //Multiplication: u = inv(K)*pinv(T)*tauScaler*tau;
    //Remark: tauScaler scales so that thrust force is given
    // between -1 and 1
    for( int i = 0; i < 6; i = i + 1 ) {
        tmp = 0;
        cout << "-----Thruster no. " << i+1 << "-----" << endl;
        for( int j = 0; j < 5; j = j + 1 ) {

            cout << "j = " << j+1 << "| tmp = " << tmp <<  endl;
            tmp = tmp + m[i][j]*input[j];

        }
        u[i] = tmp;
        cout << "value of u[" << i+1 << "] = " << u[i] << endl;
       }

    //Saturation
    for( int k = 0; k < 6; k = k + 1 ) {
        if(u[k] < -1) {
            cout << "Correcting value for u[" << k << "] = " << u[k] << " to -1." << endl;
            u[k] = -1;
        }
        if(u[k] > 1) {
            cout << "Correcting value for u[" << k << "] = " << u[k] << " to 1." << endl;
            u[k] = 1;
        }
    }

    //Print
    cout << "" << endl;
    for( int i = 0; i < 6; i = i + 1 )
        cout << "value of u[" << i+1 << "] = " << u[i] << endl;

    /*Examples
    * @input = {0.50, 0, 0, 0, 0} gives @output = [0.50, 0.50, 0.50, 0.50, 0, 0]
    * @input = {0.24, 0, 1, 0, 0} gives @output = [0.24, 0.24, 0.24, 0.24, 1, 1]
    */
}

int main()
{

    float input[] = {-0.25,0, 1, 0, 0};
    testFunction(input);

    return 0;
}

