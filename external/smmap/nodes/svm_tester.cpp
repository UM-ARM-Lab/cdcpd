#include "smmap/svm_classifier.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "smmap_planner_node");

    auto nh = std::make_shared<ros::NodeHandle>();
    auto ph = std::make_shared<ros::NodeHandle>("~");

    smmap::SVMClassifier clf(nh, ph);
    {
        Eigen::VectorXd vec(13);
        vec << 0.64493364,  0.6391668 ,  0.74615526,  0.7751439 ,  0.        , -1.        , -0.5       ,  0.        , -1.        , -0.5       , -1.        , -1.        ,  0.   ;
        std::cout << "Vec:\n" << vec << std::endl;

        auto prediction = clf.predict(vec);
        std::cout << "SKLearnPrediction: 1  CPrediction: " << prediction << std::endl;
    }

    return EXIT_SUCCESS;
}
