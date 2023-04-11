
// #include "Eigen/Dense"
#include "Helper.h"
#include "Kin.h"
#include <iostream>

using namespace std;

using namespace std;

int main(){
    Helper h;
    Kin k;
    for (float i=-5; i<5; i+=0.1){
        cout << "i: " << i << "\tsafe acos: " << k.safe_acos(i) << endl;
    }
    cout << "-----------------------------------";
    for (float i=-5; i<5; i+=0.1){
        cout << "i: " << i << "\tsafe asin: " << k.safe_asin(i) << endl;
    }
    return 0;
}
