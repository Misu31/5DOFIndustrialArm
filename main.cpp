#include <iostream>
#include <cmath>
#include <iomanip>
using namespace std;

const double PI = 3.141592653589793;
double l1 = 2;
double l2 = 2;
double l3 = 0.0;

double toRad(double deg) { return deg * (PI / 180.0); }
double toDeg(double rad) { return rad * (180.0 / PI); }

void ik(double x, double y, double z, double orientation_deg) {
    double theta = atan2(y, x);

    double r = sqrt(x * x + y * y);

    double phi = toRad(orientation_deg);

    double cos_theta2 = (pow(r, 2) + pow(z, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2);
    if (cos_theta2 > 1.0 || cos_theta2 < -1.0) {
        cout << "Error: Point (" << x << ", " << y << ", " << z << ") is out of reach!" << endl;
        return;
    }

    // Solution A
    double t2a = acos(cos_theta2);
    double t1a = atan2(z, r) - atan2(l2 * sin(t2a), l1 + l2 * cos(t2a));
    double t3a = phi - t1a - t2a;

    // Solution B
    double t2b = -acos(cos_theta2);
    double t1b = atan2(z, r) - atan2(l2 * sin(t2b), l1 + l2 * cos(t2b));
    double t3b = phi - t1b - t2b;

    cout << fixed << setprecision(2);
    cout << "theta (base rotation): " << toDeg(theta) << " deg" << endl;

    cout << "\n--- Solution A (Elbow Down) ---" << endl;
    cout << "  theta1: " << toDeg(t1a) << " deg" << endl;
    cout << "  theta2: " << toDeg(t2a) << " deg" << endl;
    cout << "  theta3: " << toDeg(t3a) << " deg" << endl;

    cout << "\n--- Solution B (Elbow Up) ---" << endl;
    cout << "  theta1: " << toDeg(t1b) << " deg" << endl;
    cout << "  theta2: " << toDeg(t2b) << " deg" << endl;
    cout << "  theta3: " << toDeg(t3b) << " deg" << endl;
}

int main() {
    double x1, y1, z1, or1;
    cout << "Enter X, Y, Z, and Orientation: ";
    if (cin >> x1 >> y1 >> z1 >> or1) {
        ik(x1, y1, z1, or1);
    }
    return 0;
}
