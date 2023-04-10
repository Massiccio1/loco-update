#include "Helper.h"

using namespace std;

int main(){
    Helper h;
    float i = 4.4;
    float o = h.dist_constrain(i);
    cout << "pre dist: " << i << "\t after: " << o << endl;
    return 0;
}
