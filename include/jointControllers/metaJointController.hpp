# ifndef METAJOINTCONTROLLER_HPP
# define METAJOINTCONTROLLER_HPP

#include <vector>

#include <Eigen/Dense>
#include <dart/dart.hpp>

# include <boost/property_tree/ptree.hpp>
# include <boost/property_tree/json_parser.hpp>

# ifndef pt_namespace
# define pt_namespace
namespace pt = boost::property_tree;
# endif

class metaJointController{

	public:
		metaJointController(const dart::dynamics::SkeletonPtr & skelPtr, const pt::ptree configurationDataTree){
			robotPtr_ = skelPtr;
			assert(robotPtr_ != nullptr);
			dt_ = robotPtr_->getTimeStep();

			configurationDataTree_ = configurationDataTree;
			Kv_ = configurationDataTree_.get<double>("jointControlConfiguration.computedTorqueControl.K_v", 0);
			Kp_ = configurationDataTree_.get<double>("jointControlConfiguration.computedTorqueControl.K_p", 0);
		}
		~metaJointController(){}

		virtual bool update(const Eigen::VectorXd &inputAcc){
			robotPtr_->setForces(computedTorque_(inputAcc));
		}
	private:

		Eigen::VectorXd computedTorque_(const Eigen::VectorXd &acc);

		double dt_;
		dart::dynamics::SkeletonPtr robotPtr_;
		pt::ptree configurationDataTree_;

		double Kv_;
		double Kp_;
};


# endif 
