#ifndef __ORTHONORMALFAMILY_H__
#define __ORTHONORMALFAMILY_H__

#include <Eigen/Eigen>

class OrthonormalFamily
{
public:
    OrthonormalFamily(const Eigen::MatrixXd family, const double epsilon);
    ~OrthonormalFamily();
    void computeOrthonormalFamily();
    const int getIndex() const;
    const Eigen::VectorXi& getOrigin() const;
    const Eigen::MatrixXd& getOnf() const;
    void setFamily(const Eigen::MatrixXd& family);
    void setEpsilon(const double epsilon);

private:
    int index;
    Eigen::VectorXi origin;
    Eigen::MatrixXd onf;
    Eigen::MatrixXd family;
    double epsilon;

};


#endif
