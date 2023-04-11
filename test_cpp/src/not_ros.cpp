
// #include "Eigen/Dense"
#include "Helper.h"
#include "Kin.h"
#include <iostream>

using namespace std;

int main(){
    Helper h;
    Kin k;
    for (float i=-50; i<50; i+=0.1){
        cout << "i: " << i << "\tsafe acos: " << k.safe_acos(i) << endl;
    }
    return 0;
}
