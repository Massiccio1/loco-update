
// #include "Eigen/Dense"
#include "Helper.h"
#include "Kin.h"
#include <iostream>

using namespace std;

using namespace std;

int main(){
    Helper h;
<<<<<<< HEAD
    Kin k;
    for (float i=-50; i<50; i+=0.1){
        cout << "i: " << i << "\tsafe acos: " << k.safe_acos(i) << endl;
    }
=======
    float i = 4.4;
    float o = h.dist_constrain(i);
    cout << "pre dist: " << i << "\t after: " << o << endl;
>>>>>>> f9c8a124153bbf17f34aebb5657493be95dba06e
    return 0;
}
