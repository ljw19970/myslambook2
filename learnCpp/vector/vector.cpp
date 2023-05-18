#include <iostream>
#include <vector>

using namespace std;

int main() {
    // 创建一个空的 vector
    vector<int> myVector;

    // 在 vector 末尾添加元素
    myVector.push_back(10);
    myVector.push_back(20);
    myVector.push_back(30);

    // 访问 vector 中的元素
    cout << "Vector elements: ";
    for (int i = 0; i < myVector.size(); i++) {
        cout << myVector[i] << " ";
    }
    cout << endl;

    // 使用迭代器遍历 vector
    cout << "Vector elements (using iterator): ";
    vector<int>::iterator it;   // 创建迭代器
    for (it = myVector.begin(); it != myVector.end(); it++) {
        cout << *it << " ";
    }
    cout << endl;

    // 使用[]访问 vector 中的元素，并修改 vector 中的元素
    myVector[1] = 25;

    // 使用 at() 访问 vector 中的元素
    cout << "Vector element at index 1: " << myVector.at(1) << endl;

    // 删除 vector 中的最后一个元素
    myVector.pop_back();

    // 检查 vector 是否为空
    if (myVector.empty()) {
        cout << "Vector is empty." << endl;
    }
    else {
        cout << "Vector size: " << myVector.size() << endl;
    }

    // 清空 vector
    myVector.clear();

    // 检查 vector 是否为空
    if (myVector.empty()) {
        cout << "Vector is empty." << endl;
    }
    else {
        cout << "Vector size: " << myVector.size() << endl;
    }

    return 0;
}