#include "../src/im_localiser-component.hpp"
#include <gtest/gtest.h>
#include <ros/init.h>

// Constants for maintaining the vector defining the super ellipse.
const unsigned int idx_center_x = 0;
const unsigned int idx_center_y = 1;
const unsigned int idx_orientation = 2;
const unsigned int idx_a= 3;
const unsigned int idx_b= 4;
const unsigned int num_ellipse_parameter = 5;

// Struct test_data containing all parameter values used in test cases.
struct test_data {
    
    Eigen::VectorXd ellipse;
    Eigen::VectorXd point;
    double expected_residual;
    double epsilon;
    double aspect_ratio;
    bool size_known;
    int test_id;

    test_data(int id) : ellipse(num_ellipse_parameter), point(2) {
        ellipse.setZero();
        point.setZero();
        expected_residual = 0.;
        epsilon = 1.;
        aspect_ratio = 0.;
        size_known = false;
        test_id = id;
    }
    static std::vector<test_data> Values() {

        // Test values for all tests.
        std::vector<test_data> values;

        // We iterate over all test cases twice
        // with size_known = {false, true}.
        bool sk = false;

        for (int i = 0; i < 2; i++) {
            if (i == 1) {
                sk = true;
            }
            
            // Test01: Residual for a point placed on the ellipse contour.
            test_data test01(values.size() + 1);
            test01.size_known = sk;
            test01.ellipse[idx_a] = 3.;
            test01.ellipse[idx_b] = 4.;
            test01.point[0] = test01.ellipse[idx_a];
            test01.point[1] = 0.;
            test01.expected_residual = 0.;
            values.push_back(test01);
            
            // Test02: Residual for a point placed on the ellipse contour.
            test_data test02(values.size() + 1);
            test02.size_known = sk;
            test02.ellipse[idx_a] = 3.;
            test02.ellipse[idx_b] = 4.;
            test02.point[0] = 0.;
            test02.point[1] = test02.ellipse[idx_b];
            test02.expected_residual = 0.;
            values.push_back(test02);

            // Test03/06: Residual for a point 1m far away from 
            // the ellipse contour.
            test_data test03(values.size() + 1);
            test03.size_known = sk;
            test03.ellipse[idx_a] = 8.;
            test03.ellipse[idx_b] = 3.;
            test03.point[0] = 0.;
            test03.point[1] = test03.ellipse[idx_b] + 1.;
            test03.expected_residual = 1.;
            values.push_back(test03);

            // Test04/08: Residual for a point on the ellipse center.
            // For this special case we decided to let the residual 
            // the minimum of both radii a, b.
            test_data test04(values.size() + 1);
            test04.size_known = sk;
            test04.ellipse[idx_a] = 5.;
            test04.ellipse[idx_b] = 6.;
            test04.point[0] = 0.;
            test04.point[1] = 0.;
            test04.expected_residual = std::min(test04.ellipse[idx_a], 
                test04.ellipse[idx_b]);
            values.push_back(test04);
     
            // Test05/10: Residual for a point on the ellipse contour
            // that has the shape of a 'Squircle'.
            test_data test05(values.size() + 1);
            test05.size_known = sk;
            test05.ellipse[idx_a] = 1.;
            test05.ellipse[idx_b] = 1.;
            test05.epsilon = 2. / 20.;
            test05.point[0] = test05.ellipse[idx_a];
            test05.point[1] = 0.1;
            test05.expected_residual = 0.;
            values.push_back(test05);
            
            // Test06/12: Residual for a point on the ellipse contour
            // that has the shape of a circle.
            test_data test06(values.size() + 1);
            test06.size_known = sk;
            test06.ellipse[idx_a] = 1.;
            test06.ellipse[idx_b] = 1.;
            test06.ellipse[idx_orientation] = 0.7;
            test06.epsilon = 2;
            test06.aspect_ratio = 1.;
            test06.point[0] = cos(test06.ellipse[idx_orientation]);
            test06.point[1] = sin(test06.ellipse[idx_orientation]);
            test06.expected_residual = 0.;
            values.push_back(test06);
        }

        return values;
    }
};

// Implementation of a toString() for nice prints of test_data.
std::ostream& operator<< (std::ostream& os, const test_data& data) {
    os << "{test_id =" << data.test_id 
        << ", ellipse=[" << data.ellipse.transpose() 
        << "], point=[" << data.point.transpose() 
        << ", aspect_ratio=" << data.aspect_ratio
        << ", epsilon=" << data.epsilon
        << ", size_known=" << data.size_known
        << ", expected_residual=" << data.expected_residual
        << "}";
    return os;
}

/* Testscases for im_localiser-helperfunctions.cpp .
 *
 */
class Residual2dTest : public ::testing::TestWithParam<test_data> {
    public:
        Residual2dTest() : 
            localiser("Test")
            {}
    protected:
        virtual void SetUp() {
        }
        Im_localiser localiser;
};

/**
 * Setup for the test. 
 */
TEST_P(Residual2dTest, getResiduals2d) {

    test_data data = GetParam();
   
    double residual = localiser.getResidual2d(data.ellipse, 
                                                data.point,
                                                data.epsilon, 
                                                data.aspect_ratio, 
                                                data.size_known);

    double expected_residual = data.expected_residual;
    ASSERT_DOUBLE_EQ(expected_residual, residual);
}

INSTANTIATE_TEST_CASE_P(LocaliserResidual2dParameterTest, Residual2dTest,
                        ::testing::ValuesIn(test_data::Values()));
/* 
 * Run all the tests.
 * TODO: malteim: remove the ros::init as soon as #239 gets merged.
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "test_im_localiser_residual_2d", 
        ros::init_options::AnonymousName);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
