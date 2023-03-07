#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//�������ʽ�Ľ׳ˣ�input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

// ��ȡhession����Ҳ���Ǿ���Q
void TrajectoryGeneratorWaypoint::GetHession(const int n_seg,
                    const int d_order,
                    const Eigen::VectorXd& Time,
                    Eigen::SparseMatrix<double>& hession) 
{

    int p_order = 2 * d_order - 1; //����
    int p_num1d = p_order + 1; //ϵ������
    hession.resize(n_seg * p_num1d, n_seg * p_num1d);

    hession.setZero(); //��ʼ��Q����

    for (int k = 0; k < n_seg; ++k) {

        for (int i = d_order; i < p_num1d; ++i) {
            for (int j = d_order; j < p_num1d; ++j) {
                double value = 1.0 * Factorial(i) / Factorial(i - d_order) * Factorial(j)
                    / Factorial(j - d_order) / (i + j - 2 * d_order + 1) * pow(Time(k), i + j - 2 * d_order + 1);
                // SparseMatrix.insert�᷵��һ��ֵ�����ã��������������
                hession.insert(k * p_num1d + i, k * p_num1d + j) = value;
            }
        }
        
    }
}

// ������Լ�������ָ��λ�ò���ϵ��
void TrajectoryGeneratorWaypoint::InsertCoff(const int row,
                                            const int col,
                                            Eigen::SparseMatrix<double>& linearMatrix,
                                            const double t,
                                            const int d_order,
                                            bool one_line,
                                            bool reverse) 
{

    int p_num1d = 2 * d_order;

    int flag = d_order;
    if (one_line) { // Ӧ����������ֻȥ��һ�м�λ����Ϣ�ģ����м�Լ��ֻ��λ�õ����ֻ��Ҫ��һ��
        flag = 1;
    }

    Eigen::MatrixXd coff(d_order, p_num1d);

    if (d_order == 4) {
        coff << 1.0, 1.0 * t, 1.0 * pow(t, 2), 1.0 * pow(t, 3), 1.0 * pow(t, 4), 1.0 * pow(t, 5), 1.0 * pow(t, 6), 1.0 * pow(t, 7),
            0.0, 1.0, 2.0 * t, 3.0 * pow(t, 2), 4.0 * pow(t, 3), 5.0 * pow(t, 4), 6.0 * pow(t, 5), 7.0 * pow(t, 6),
            0.0, 0.0, 2.0, 6.0 * t, 12.0 * pow(t, 2), 20.0 * pow(t, 3), 30.0 * pow(t, 4), 42.0 * pow(t, 5),
            0.0, 0.0, 0.0, 6.0, 24.0 * t, 60.0 * pow(t, 2), 120.0 * pow(t, 3), 210.0 * pow(t, 4);
    }
    else if (d_order == 3) {
        coff << 1.0, 1.0 * t, 1.0 * pow(t, 2), 1.0 * pow(t, 3), 1.0 * pow(t, 4), 1.0 * pow(t, 5),
            0.0, 1.0, 2.0 * t, 3.0 * pow(t, 2), 4.0 * pow(t, 3), 5.0 * pow(t, 4),
            0.0, 0.0, 2.0, 6.0 * t, 12.0 * pow(t, 2), 20.0 * pow(t, 3);
    }
    else {
        cout << "��ʱֻ֧��minisnap��minijerk";
    }

    if (reverse) { //ȫ��ȡ����Ϊ�˸�������������׼��
        coff = coff * (-1.0);
    }

    for (int i = 0; i < d_order && i < flag; ++i) {
        for (int j = 0; j < p_num1d; ++j) {
            linearMatrix.insert(row + i, col + j) = coff(i, j);
        }
    }

}

// ��ȡ��ʽԼ������Ҳ���Ǿ���Aeq
void TrajectoryGeneratorWaypoint::GetLinearConstraintsMatrix(const int n_seg,
                                                        const int d_order,
                                                        const Eigen::VectorXd& Time,
                                                        Eigen::SparseMatrix<double>& linearMatrix) 
{

    int p_order = 2 * d_order - 1;
    int p_num1d = p_order + 1;

    cout << "p_num1d:" << p_num1d << endl;;
    cout << "n_seg:" << n_seg << endl;
    linearMatrix.resize(2 * d_order + (n_seg - 1) * (d_order + 1), p_num1d * n_seg);

    // �����յ�����Լ��
    int row = 0;
    int col = 0;
    InsertCoff(row, col, linearMatrix, 0, d_order, false, false);
    cout << "row:" << row << endl;

    row += d_order;
    col = (n_seg - 1) * p_num1d;
    cout << "row:" << row << endl;
    cout << "col:" << col << endl;
    InsertCoff(row, col, linearMatrix, Time(n_seg - 1), d_order, false, false);


    // �м�ڵ��λ��Լ��
    row += d_order;
    for (int k = 0; k < n_seg - 1; ++k) {
        InsertCoff(row + k, k * p_num1d, linearMatrix, Time(k), d_order, true, false);
    }
    cout << "row:" << row << endl;
    // ������Լ��
    row += n_seg - 1;
    for (int k = 0; k < n_seg - 1; ++k) {
        InsertCoff(row, k * p_num1d, linearMatrix, Time(k), d_order, false, false);//ǰ��ĩβ
        InsertCoff(row, (k + 1) * p_num1d, linearMatrix, 0, d_order, false, true);//��ο�ͷ
        row += d_order;
    }
    cout << "row:" << row << endl;

}



/*

    ����1�������� closed-form

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    
            const Eigen::MatrixXd &Path,          
            const Eigen::MatrixXd &Vel,           
            const Eigen::MatrixXd &Acc,           
            const Eigen::VectorXd &Time)          
{
    
    int p_order   = 2 * d_order - 1;             
    int p_num1d   = p_order + 1;                  

    int m = Time.size();                        //��segment�Ķ���
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // ��Ϊ����Ҫ��(x,y,z)�ֱ���⣬����Ҫ3���Ĳ�����
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);


    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
    
    // ����Q
    MatrixXd Q = MatrixXd::Zero(p_num1d * m, p_num1d * m);
    for (int k = 0; k < m; ++k) {
        MatrixXd Q_k = MatrixXd::Zero(p_num1d, p_num1d);
        for (int i = d_order; i < p_num1d; ++i) {
            for (int j = d_order; j < p_num1d; ++j) {
                Q_k(i, j) = 1.0 * Factorial(i) / Factorial(i - d_order) * Factorial(j) / Factorial(j - d_order) / (i + j - 2 * d_order + 1) * pow(Time(k), i + j - 2 * d_order + 1);
            }
        }
        Q.block(k * p_num1d, k * p_num1d, p_num1d, p_num1d) = Q_k; //�������
    }
    
    // ����M
    MatrixXd M = MatrixXd::Zero(m * p_num1d, m * p_num1d); //����8������ʽϵ��ӳ�䵽8��������
    MatrixXd coeff(d_order, p_num1d);
    coeff << 1, 1, 1, 1, 1, 1, 1, 1,
        0, 1, 2, 3, 4, 5, 6, 7,
        0, 0, 2, 6, 12, 20, 30, 42,
        0, 0, 0, 6, 24, 60, 120, 210;


    for (int k = 0; k < m; k++) {

        MatrixXd M_k = MatrixXd::Zero(p_num1d, p_num1d);

        double t = Time(k);
        for (int i = 0; i < d_order; i++) {
            M_k(i, i) = coeff(i, i);
        }

        for (int i = 0; i < d_order; i++) {
            for (int j = i; j < p_num1d; j++) {
                    M_k(i + d_order, j) = coeff(i, j) * pow(t, j - i);
            }
        }

        M.block(k * p_num1d, k * p_num1d, p_num1d, p_num1d) = M_k;
    }



    // ����C_t
    int ct_rows = d_order * 2 * m; //d������ 2*4*seg
    int ct_cols = d_order * 2 * m - (m - 1) * d_order; // df+dp������ 4*(seg+1)
    MatrixXd Ct = MatrixXd::Zero(ct_rows, ct_cols);
    // Ϊd�������ұ�
    vector<int> d_vector;
    for (int k = 0; k < m; k++) {
        for (int t = 0; t < 2; t++) {
            for (int d = 0; d < d_order; d++) {
                d_vector.push_back(k * 100 + t * 10 + d);
            }
        }
    }
    int val, row;
    int col = 0;

    // �̶����״̬
    int k = 0;
    int t = 0;
    int d = 0;
    for (d = 0; d < d_order; d++) {
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it); // �����������ľ���
        Ct(row, col) = 1;
        col += 1;
    }
    // �̶��м�ڵ�λ��
    t = 1;
    d = 0;
    for (k = 0; k < m - 1; k++) {
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;

        val = (k + 1) * 100 + (t - 1) * 10 + d;
        it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;

        col += 1;
    }

    // �̶��յ�״̬
    k = m - 1;
    t = 1;
    d = 0;
    for (d = 0; d < d_order; d++) {
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;
        col += 1;
    }

    // ��֤������Լ��
    k = 0;
    t = 1;
    d = 1;
    for (k = 0; k < m - 1; k++) {
        for (d = 1; d < d_order; d++) {
            val = k * 100 + t * 10 + d;
            auto it = std::find(d_vector.begin(), d_vector.end(), val);
            row = std::distance(d_vector.begin(), it);
            Ct(row, col) = 1;

            val = (k + 1) * 100 + (t - 1) * 10 + d;
            it = std::find(d_vector.begin(), d_vector.end(), val);
            row = std::distance(d_vector.begin(), it);
            Ct(row, col) = 1;

            col += 1;
        }

    }


    MatrixXd C = Ct.transpose();
    MatrixXd M_inv = M.inverse();
    MatrixXd M_inv_t = M_inv.transpose();
    MatrixXd R = C * M_inv_t * Q * M_inv * Ct;


    // �ֱ�������������Ӧ��ϵ��

    int num_d_F = 2 * d_order + m - 1;
    int num_d_P = (m - 1) * (d_order - 1);

    MatrixXd R_pp = R.bottomRightCorner(num_d_P, num_d_P);


    MatrixXd R_fp = R.topRightCorner(num_d_F, num_d_P);


    for (int dim = 0; dim < 3; dim++) {

        VectorXd wayPoints = Path.col(dim);
        VectorXd d_F = VectorXd::Zero(num_d_F);

        // �̶����״̬
        d_F(0) = wayPoints(0);

        // �̶��м�ڵ�λ��
        for (int i = 0; i < m - 1; i++) {
            d_F(d_order + i) = wayPoints(i + 1);
        }

        // �̶��յ�״̬
        d_F(d_order + m - 1) = wayPoints(m);


        VectorXd d_P = -1.0 * R_pp.inverse() * R_fp.transpose() * d_F; //  ͨ����ʽ���d_P

        VectorXd d_total(d_F.rows() + d_P.rows());

        d_total << d_F, d_P;

        VectorXd poly_coef_1d = M.inverse() * Ct * d_total; // ������������ʽϵ��

        MatrixXd poly_coef_1d_t = poly_coef_1d.transpose();


        for (int k = 0; k < m; k++) {
            PolyCoeff.block(k, dim * p_num1d, 1, p_num1d) = poly_coef_1d_t.block(0, k * p_num1d, 1, p_num1d);
        }
    }

    return PolyCoeff;
}

/*

    �ڶ��ֽⷨ:ͨ���������ֵ��

*/

// �ع�һ�º��������˸�Solver�����
Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
                                                                const int d_order,                    
                                                                const Eigen::MatrixXd& Path,          
                                                                const Eigen::MatrixXd& Vel,           
                                                                const Eigen::MatrixXd& Acc,           
                                                                const Eigen::VectorXd& Time,
                                                                OsqpEigen::Solver& slover
)        
{
    
    int p_order = 2 * d_order - 1;                              
    int p_num1d = p_order + 1;                  

    int m = Time.size();                               


    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);          

    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);


    //slover.settings()->setWarmStart(true);


    // ���ñ�������
    slover.data()->setNumberOfVariables(m * p_num1d);

    //����Լ������
    slover.data()->setNumberOfConstraints(d_order * 2 + (m - 1) * (d_order + 1));


    // ����H����
    Eigen::SparseMatrix<double> hession;
    GetHession(m, d_order, Time, hession);
    if (!slover.data()->setHessianMatrix(hession)) {
        cout << "����hession����ʧ��";
        return Eigen::MatrixXd::Zero(1, 1);
    }
    else {
        cout << "hession�������óɹ�" << endl;
    }

    //��������Լ������
    Eigen::SparseMatrix<double> linearMatrix;
    GetLinearConstraintsMatrix(m, d_order, Time, linearMatrix);

    if (!slover.data()->setLinearConstraintsMatrix(linearMatrix)) {
        cout << "����Linear����ʧ��";
        return Eigen::MatrixXd::Zero(1, 1);
    }
    else {
        cout << "Linear�������óɹ�" << endl;
    };



    Eigen::VectorXd gradient(p_num1d * m);
    gradient.setZero();

    // �����ݶ�Լ��
    if (!slover.data()->setGradient(gradient)) {
        cout << "�ݶ�����ʧ��" << endl;
    }
    else {
        cout << "�ݶ����óɹ�" << endl;
    }



    // ���ñ߽磬������⣬����������Լ������ʽ����Ū��һ���ľ��ǵ�ʽ������
    Eigen::VectorXd lowbound = VectorXd::Zero(d_order * 2 + (m - 1) * (d_order + 1));
    Eigen::VectorXd upbound = VectorXd::Zero(d_order * 2 + (m - 1) * (d_order + 1));

    slover.data()->setLowerBound(lowbound);
    slover.data()->setUpperBound(upbound);

    //��ʼ�������
    if (!slover.isInitialized()) {
        slover.initSolver();
    }


    for (int dim = 0; dim < 3; dim++) {

        VectorXd wayPoints = Path.col(dim);


        // ���λ��
        lowbound(0) = wayPoints(0);
        upbound(0) = wayPoints(0);

        // �յ�λ��
        lowbound(d_order) = wayPoints(m);
        upbound(d_order) = wayPoints(m);

        // �̶��м�ڵ�λ��
        for (int i = 0; i < m - 1; i++) {
            lowbound(2 * d_order + i) = wayPoints(i + 1);
            upbound(2 * d_order + i) = wayPoints(i + 1);
        }

        // ���±߽�
        slover.updateBounds(lowbound, upbound);

        // ���
        slover.solve();

        Eigen::VectorXd poly_coef_1d = slover.getSolution();


        MatrixXd poly_coef_1d_t = poly_coef_1d.transpose();


        for (int k = 0; k < m; k++) {
            PolyCoeff.block(k, dim * p_num1d, 1, p_num1d) = poly_coef_1d_t.block(0, k * p_num1d, 1, p_num1d);
        }


    }

    // ÿ�ε���֮����Ҫ�������
    slover.data()->clearHessianMatrix();
    slover.data()->clearLinearConstraintsMatrix();
    slover.clearSolverVariables();
    slover.clearSolver();

    return PolyCoeff;
}