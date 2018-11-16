# ifndef METACONTROLLER_H
# define METACONTROLLER_H


#include <vector>

#include <Eigen/Dense>

#include <dart/dart.hpp>


# include <boost/property_tree/ptree.hpp>
# include <boost/property_tree/json_parser.hpp>

# include <jointControllers/metaJointController.hpp> 

# ifndef pt_namespace
# define pt_namespace
namespace pt = boost::property_tree;
# endif


class metaController{
	public:
		metaController(const dart::dynamics::SkeletonPtr & skelPtr, const pt::ptree configurationDataTree);
		~metaController(){}
		//virtual Eigen::VectorXd computeTorques(); 

		virtual bool update(){
		//	robotPtr_->setForces(computeTorques());
			Eigen::VectorXd tempAcc(robotPtr_->getNumDofs());
			tempAcc.setZero();
			jointControllerPtr_->update(tempAcc);
			//robotPtr_->setForces(accToTorque(tempAcc));
		}
		
		
		//Eigen::VectorXd accToTorque(const Eigen::VectorXd &acc);

		int readControlMode() const{
			return controlMode_;	
		}

	protected:
		bool enabled_;
		dart::dynamics::SkeletonPtr robotPtr_;
		double dt_;

		pt::ptree configurationDataTree_;
		std::shared_ptr<metaJointController> jointControllerPtr_;

		int controlMode_;

		/**
		 * We use the computed torque control to realize a desired joint acceleration.
		 * @param acc The desired joint accleration.
		 * @return The corresponding torque 
		 */
		// Eigen::VectorXd computedTorque_(const Eigen::VectorXd &acc);
		/**
		 * We use the computed torque control to realize a desired joint acceleration.
		 * @param acc The desired joint accleration.
		 * @return The corresponding torque 
		 */
		// Eigen::VectorXd jointVelocityControl_(const Eigen::VectorXd &acc);

};
# endif
