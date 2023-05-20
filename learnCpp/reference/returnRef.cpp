#include <iostream>

using namespace std;

double vals[] = {10.1, 12.6, 33.1, 24.1, 50.0};

double &setValues(int i) {
    double &ref = vals[i];
    return ref; // 返回第 i 个元素的引用，ref 引用 vals[i]
}

int main() {
    cout << "vals[1] = " << vals[1] << endl;
    setValues(1) = 20.23; // 改变第 2 个元素

    cout << "vals[1] = " << vals[1] << endl;
    return 0;
}