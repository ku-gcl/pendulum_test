#include <cmath>
#include <iostream>

// g++ -o test/matrix test/matrix.cpp && sudo test/matrix

using namespace std;

float A[2][2] = {{1, 2}, {3, 4}};

int main() {
    cout << "m1[i]" << endl;
    float *m1 = A[0];
    for (int i = 0; i < 4; i++) {
        printf("%f \n", m1[i]);
    }

    cout << endl << "m1[x]+i" << endl;
    for (int i = 0; i < 4; i++) {
        printf("%f \n", m1[0]+i);
    }

    cout << endl << "m1" << endl;
    cout << endl << m1 << endl;
    for (int i = 0; i < 4; i++) {
        printf("%p \n", (void*) (m1+i));
    }

    // ---------------
    cout << "A[0]: " << *A[0] << endl;
    // cannot use
    // cout << "A[0][0]: " << *(A[0][0]) << endl;
    cout << "A[0][1]: " << A[0][1] << endl;
    cout << "A[1]: " << *A[1] << endl;
    cout << "A[2]: " << *A[2] << endl;

    cout << "A matrix:" << endl;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            cout << A[i][j] << " ";
        }
        cout << endl; // 行を変える
    }

    return 0;
}
